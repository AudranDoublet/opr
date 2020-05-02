extern crate image_manipulation;
extern crate rayon;
extern crate serde_yaml;
extern crate tobj;

use std::collections::HashMap;
use std::error::Error;
use std::fs::File;
use std::path::Path;

use nalgebra::{UnitQuaternion, Vector2, Vector3};
use rand::*;
use rand::distributions::Uniform;
use rayon::prelude::*;
use search::{BVH, BVHParameters, Ray};

use crate::*;

pub struct Scene
{
    camera: Camera,
    triangles: Vec<shapes::Triangle>,
    materials: Vec<Material>,
    textures: Vec<Texture>,
    lights: Vec<Light>,
    light_ambient: Vector3<f32>,
    correction_bias: f32,
    air_ior: f32,
    tree: BVH<shapes::Triangle>,
}

fn load_texture(root: &Path, path: &Path) -> Result<Texture, Box<dyn Error>>
{
    let (width, height, pixels) = load_image(&root.join(path))?;
    let tex = Texture::new(width, height, pixels);

    Ok(tex)
}

fn clamp_light(v: &Vector3<f32>) -> Vector3<f32> {
    Vector3::new(
        v.x.min(1.0).max(0.0),
        v.y.min(1.0).max(0.0),
        v.z.min(1.0).max(0.0),
    )
}

fn reflect(ray: &Ray, normal: &Vector3<f32>) -> Vector3<f32> {
    &ray.direction - 2.0 * normal.dot(&ray.direction) * normal
}

fn refract(ray: &Ray, normal: &Vector3<f32>, cos_i: f32, n1: f32, n2: f32) -> Vector3<f32> {
    let n = n1 / n2;
    let k = 1.0 - n * n * (1.0 - cos_i * cos_i);
    assert!(k >= 0.0, "total internal reflection");
    n * &ray.direction + (n * cos_i - k.sqrt()) * normal
}

fn refract_unknown_if_tir(ray: &Ray, normal: &Vector3<f32>, cos_i: f32, n1: f32, n2: f32) -> Option<Vector3<f32>> {
    let n = n1 / n2;
    let k = 1.0 - n * n * (1.0 - cos_i * cos_i);
    if k < 0.0 {
        None
    } else {
        Some(n * &ray.direction + (n * cos_i - k.sqrt()) * normal)
    }
}

#[cfg(not(use_fresnel_approximation))]
fn reflectivity_coeff(cos_i: f32, n1: f32, n2: f32) -> f32 {
    // Fresnel
    let sin_t = (n1 / n2) * (1.0 - cos_i * cos_i).sqrt();
    if sin_t > 1.0 { return 1.0; } // total internal reflection
    let cos_t = 1. - sin_t * sin_t;
    let r_par = ((n1 * cos_i - n2 * cos_t) / (n1 * cos_i + n2 * cos_t)).powi(2);
    let r_orth = ((n2 * cos_i - n1 * cos_t) / (n2 * cos_i + n1 * cos_t)).powi(2);

    (r_orth + r_par) / 2.0
}

#[cfg(use_fresnel_approximation)]
fn reflectivity_coeff(cos_i: f32, n1: f32, n2: f32) -> f32 {
    // Schlick: Fresnel approximation
    let r0 = ((n1 - n2) / (n1 + n2)).powi(2);
    let cos_i = if n1 > n2 {
        let n = n1 / n2;
        let sin_t2 = n * n * (1.0 - cos_i * cos_i);
        if sin_t2 > 1.0 { return 1.0; }
        (1.0 - sin_t2).sqrt()
    } else { cos_i };
    let x = 1.0 - cos_i;
    r0 + (1.0 - r0) * x.powi(5)
}

fn beer_attenuation(transmittance: &Vector3<f32>, distance: f32) -> Vector3<f32> {
    Vector3::new(
        (-transmittance.x * distance).exp(),
        (-transmittance.y * distance).exp(),
        (-transmittance.z * distance).exp(),
    )
}

impl Scene {
    pub fn new() -> Scene {
        Scene {
            camera: Camera::new_empty(),
            triangles: Vec::new(),
            materials: Vec::new(),
            textures: Vec::new(),
            lights: Vec::new(),
            light_ambient: Vector3::zeros(),
            correction_bias: 0.,
            air_ior: 1.,
            tree: BVH::default(),
        }
    }

    pub fn from_config(file: scene_config::SceneConfig, default_material: &Path) -> Result<Scene, Box<dyn Error>> {
        let mut scene = Scene::new();

        scene.load_mtl(default_material)?[0];

        for obj in file.objects {
            let r = UnitQuaternion::from_euler_angles(obj.rotation.x, obj.rotation.y, obj.rotation.z);
            scene.load_obj(Path::new(&obj.path), obj.position, r, obj.scale, obj.override_material)?;
        }

        for mut light in file.lights {
            light.init();
            match light {
                Light::AmbientLight { color, .. } => scene.light_ambient += color,
                _ => scene.add_light(light)
            }
        }
        scene.light_ambient = clamp_light(&scene.light_ambient);

        scene.camera = file.camera;

        Ok(scene)
    }

    pub fn from_file(path: &Path, default_material: &Path) -> Result<Scene, Box<dyn Error>> {
        let file: scene_config::SceneConfig = serde_yaml::from_reader(File::open(path)?)?;

        Scene::from_config(file, default_material)
    }

    pub fn load_mtl(&mut self, path: &Path) -> Result<Vec<usize>, Box<dyn Error>> {
        let root = path.parent().ok_or("bad path")?;
        let (materials, _) = tobj::load_mtl(path)?;

        let mut result = Vec::new();
        let mut map_texture = HashMap::new();

        if self.textures.len() == 0 {
            map_texture.insert("", 0);
            self.textures.push(Texture::new_empty());
        }

        for mat in materials.iter()
        {
            if mat.diffuse_texture != ""
            {
                let tex = load_texture(root, &Path::new(&mat.diffuse_texture))?;

                self.textures.push(tex);
                map_texture.insert(&mat.diffuse_texture, self.textures.len() - 1);
            }

            result.push(self.materials.len());
            self.materials.push(Material::new(&mat, &map_texture));
        }

        Ok(result)
    }

    pub fn load_obj(&mut self, path: &Path, position: Vector3<f32>, rotation: UnitQuaternion<f32>, scale: Vector3<f32>, override_material: Option<String>) -> Result<(), Box<dyn Error>> {
        let root = path.parent().ok_or("bad path")?;
        let (models, materials) = tobj::load_obj(path)?;

        let override_material_id = if let Some(material_path) = override_material {
            Some(self.load_mtl(Path::new(&material_path))?[0])
        } else { None };

        let mat_id_app = self.materials.len();

        let mut map_texture = HashMap::new();

        if self.textures.len() == 0 {
            map_texture.insert("", 0);
            self.textures.push(Texture::new_empty());
        }

        for mat in materials.iter()
        {
            if mat.diffuse_texture != ""
            {
                let tex = load_texture(root, &Path::new(&mat.diffuse_texture))?;

                self.textures.push(tex);
                map_texture.insert(&mat.diffuse_texture, self.textures.len() - 1);
            }

            self.materials.push(Material::new(&mat, &map_texture));
        }

        for model in models.iter()
        {
            let mesh = &model.mesh;
            let mat_id = if let Some(id) = override_material_id { id } else {
                if let Some(mat) = mesh.material_id {
                    mat + mat_id_app
                } else { 0 }
            };

            let mut vertices = vec![];

            for i in (0..mesh.positions.len()).step_by(3)
            {
                let pos = rotation * Vector3::new(mesh.positions[i + 0],
                                                  mesh.positions[i + 1],
                                                  mesh.positions[i + 2]).component_mul(&scale) + position;
                vertices.push(pos);
            }

            let mut vertices_normal = vec![];

            for i in (0..mesh.normals.len()).step_by(3)
            {
                vertices_normal.push(rotation * Vector3::new(mesh.normals[i + 0],
                                                             mesh.normals[i + 1],
                                                             mesh.normals[i + 2]));
            }

            let mut vertices_tex = vec![Vector2::zeros(); vertices.len()];

            for i in (0..mesh.texcoords.len()).step_by(2)
            {
                vertices_tex[i / 2] = Vector2::new(mesh.texcoords[i + 0],
                                                   mesh.texcoords[i + 1]);
            }

            for i in (0..mesh.indices.len()).step_by(3)
            {
                let a = mesh.indices[i + 0] as usize;
                let b = mesh.indices[i + 1] as usize;
                let c = mesh.indices[i + 2] as usize;

                let normal = (vertices[b] - vertices[a]).cross(&(vertices[c] - vertices[a]));

                self.triangles.push(
                    shapes::Triangle::new(vertices[a],
                                          vertices[b],
                                          vertices[c],
                                          *vertices_normal.get(a).unwrap_or(&normal),
                                          *vertices_normal.get(b).unwrap_or(&normal),
                                          *vertices_normal.get(c).unwrap_or(&normal),
                                          vertices_tex[a],
                                          vertices_tex[b] - vertices_tex[a],
                                          vertices_tex[c] - vertices_tex[a],
                                          mat_id,
                                          self.triangles.len(),
                    ));
            }
        }

        Ok(())
    }

    pub fn add_triangles(&mut self, triangles: &Vec<shapes::Triangle>) {
        for v in triangles {
            self.triangles.push(*v);
        }
    }

    pub fn add_light(&mut self, light: Light) {
        self.lights.push(light);
    }

    pub fn setup_camera(&mut self, origin: Vector3<f32>, up: Vector3<f32>, front: Vector3<f32>) {
        self.camera = Camera::new(origin, up, front, 512., 512.)
    }

    pub fn build(&mut self, max_depth: u32) {
        self.tree = BVH::build_params(&BVHParameters {
            max_depth: max_depth as usize,
            max_shapes_in_leaf: 1,
            bucket_count: 6,
        }, &self.triangles)
    }

    fn sky_color(&self, _ray: Ray) -> Vector3<f32> {
        _ray.direction.apply_into(|f| f.cos())
        // Vector3::zeros()
        // Vector3::new(0.65, 0.65, 0.65)
    }

    fn compute_diffuse_and_specular(&self, position: &Vector3<f32>, normal: &Vector3<f32>, shininess: f32, mut id_triangle_from: usize) -> (Vector3<f32>, Vector3<f32>) {
        let mut diffuse = Vector3::zeros();
        let mut specular = Vector3::zeros();
        self.lights.iter().filter(|l| match l.shadow_ray(*position) {
            Some(mut v) => {
                let mut has_interference = false;
                while let Some((intersection, triangle)) = self.tree.ray_intersect_with_predicate(&v, &|t| t.id != id_triangle_from) {
                    let material = &self.materials[triangle.material];
                    has_interference = material.illumination_model < 5; // hack: we just ignore transparent objects
                    if has_interference {
                        break;
                    }
                    let intersection_position = intersection.position(&v);
                    v = Ray::new(intersection_position + v.direction * self.correction_bias, v.direction);
                    id_triangle_from = triangle.id;
                }
                !has_interference
            }
            None => true,
        }).for_each(|l| {
            let light_color = &l.get_color();
            let light_direction = &l.get_direction(position);

            let h = (light_direction + self.camera.get_origin() - position).normalize();
            let specular_intensity = normal.dot(&h).max(0.0).powf(shininess);

            diffuse += light_color * normal.dot(light_direction).max(0.);
            specular += light_color * specular_intensity;
        });

        (diffuse, specular)
    }

    fn get_juncture_information(&self, ray: &Ray, normal: Vector3<f32>, material: &Material, is_inside: &mut bool) -> (f32, Vector3<f32>, f32, f32) {
        let cos_i = normal.dot(&ray.direction);
        if cos_i > 0.0 {
            *is_inside = true;
            (cos_i, -normal, material.optical_density, self.air_ior)
        } else {
            *is_inside = false;
            (-cos_i, normal, self.air_ior, material.optical_density)
        }
    }

    fn cast_ray(&self, ray: Ray, max_rec: u8, mut distance_inside_medium: f32, id_triangle_from: usize) -> Vector3<f32> {
        if let Some((i, triangle)) = self.tree.ray_intersect_with_predicate(&ray, &|t| t.id != id_triangle_from) {
            let normal = (triangle.v1_normal * (1.0 - i.u - i.v)
                + triangle.v2_normal * i.u
                + triangle.v3_normal * i.v).normalize();


            let material = &self.materials[triangle.material];

            let ka = &material.get_ambient();
            let kd = &material.get_diffuse(&self.textures, &triangle, i.u, i.v);
            let ks = &material.specular;

            if material.illumination_model == 0 {
                return clamp_light(kd);
            }

            let position = i.position(&ray);
            let (diffuse, mut specular) = self.compute_diffuse_and_specular(&position, &normal, material.shininess, id_triangle_from);
            let mut color = Vector3::zeros();

            distance_inside_medium += i.distance;

            let reflected_color = if material.illumination_model >= 3 {
                let reflect_direction = reflect(&ray, &normal);
                let reflect_ray = Ray::new(position + &reflect_direction * self.correction_bias, reflect_direction);
                if max_rec == 0 { self.sky_color(reflect_ray) } else {
                    self.cast_ray(reflect_ray, max_rec - 1, distance_inside_medium, triangle.id)
                }
            } else { Vector3::zeros() };
            let mut refract_direction = None;
            let mut coeff_reflectivity = 0.0;
            let mut is_inside = false;

            match material.illumination_model {
                1 => specular = Vector3::zeros(),
                2 => (),
                3 | 4 => specular += reflected_color,
                5 => {
                    let (cos_i, _, n1, n2) = self.get_juncture_information(&ray, normal, &material, &mut is_inside);
                    specular += reflected_color * reflectivity_coeff(cos_i, n1, n2);
                }
                6 => {
                    let (cos_i, normal, n1, n2) = self.get_juncture_information(&ray, normal, &material, &mut is_inside);
                    refract_direction = refract_unknown_if_tir(&ray, &normal, cos_i, n1, n2);
                    specular += reflected_color;
                }
                7 => {
                    let (cos_i, normal, n1, n2) = self.get_juncture_information(&ray, normal, &material, &mut is_inside);
                    coeff_reflectivity = reflectivity_coeff(cos_i, n1, n2);
                    if coeff_reflectivity < 0.99 {
                        refract_direction = Some(refract(&ray, &normal, cos_i, n1, n2));
                    }

                    specular += reflected_color * coeff_reflectivity;
                }
                _ => panic!(format!("Unknown illum model {}", material.illumination_model))
            };

            let beer = if is_inside {
                beer_attenuation(&material.transmission, distance_inside_medium)
            } else { Vector3::new(1., 1., 1.) };

            if let Some(refract_direction) = refract_direction {
                let refract_ray = Ray::new(&position + &refract_direction * self.correction_bias, refract_direction);
                let refract_color = if max_rec == 0 {
                    self.sky_color(refract_ray)
                } else {
                    self.cast_ray(refract_ray, max_rec - 1, 0.0, triangle.id)
                };

                if material.illumination_model == 6 {
                    color += (Vector3::new(1.0, 1.0, 1.0) - ks).component_mul(&refract_color);
                } else {
                    color += (1.0 - coeff_reflectivity) * refract_color;
                }
            }

            color += ka.component_mul(&self.light_ambient) + kd.component_mul(&diffuse) + ks.component_mul(&specular);

            clamp_light(&color.component_mul(&beer))
        } else {
            self.sky_color(ray)
        }
    }

    fn grad_map(&self, image: &Vec<Vector3<f32>>, width: usize, height: usize) -> Vec<f32> {
        image_manipulation::Image::from_vectors(&image, width, height)
            .schnarr()
            .to_grayscale()
            .pixels.into_par_iter().map(|p| p as f32 / 255.0).collect()
    }

    fn apply_anti_aliasing(&self, image: &mut Vec<Vector3<f32>>, width: usize, height: usize, max_rec: u8, max_sample: f32) {
        let grad_map = self.grad_map(image, width, height);
        let threshold = 1. / (max_sample + 1.0);

        image.par_iter_mut().enumerate()
            .filter(|(i, _)| grad_map[*i] > threshold)
            .for_each(|(i, p)| {
                let mut rng = rand::thread_rng();
                let distribution = Uniform::new_inclusive(-0.5, 0.5);

                let (x, y) = ((i % width) as f32, (i / width) as f32);
                let nb_sample = (grad_map[i] * max_sample) as usize;

                for _ in 0..nb_sample {
                    let ray = self.camera.generate_ray(x + rng.sample(distribution), y + rng.sample(distribution));
                    *p += self.cast_ray(ray, max_rec, 0.0, usize::max_value());
                }

                *p /= (nb_sample + 1) as f32;
            });
    }

    pub fn render(&mut self, width: usize, height: usize, max_rec: u8, anti_aliasing_max_sample: usize) -> image_manipulation::Image {
        self.camera.set_size(width as f32, height as f32);

        let mut pixels = (0..width * height).into_par_iter()
            .map(|i| self.cast_ray(self.camera.generate_ray((i % width) as f32, (i / width) as f32), max_rec, 0.0, usize::max_value()))
            .collect();

        if anti_aliasing_max_sample > 0 {
            self.apply_anti_aliasing(&mut pixels, width, height, max_rec, anti_aliasing_max_sample as f32);
        }

        image_manipulation::Image::from_vectors(&pixels, width, height)
    }
}
