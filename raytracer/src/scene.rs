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
use search::{BVH, BVHParameters, Ray, IntersectsBVHShape, Intersection};

use crate::scene_config::*;
use crate::*;
use crate::shapes::{Shape, Plane, Sphere};

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

type Object = Box<dyn Shape + Sync + Send>;

pub struct Scene
{
    camera: Camera,
    planes: Vec<Plane>,
    objects: Vec<Object>,
    materials: Vec<Material>,
    shaders: Vec<Box<dyn FragmentShader>>,
    textures: Vec<Texture>,
    lights: Vec<Light>,
    light_ambient: Vector3<f32>,
    default_material_id: usize,
    correction_bias: f32,
    correction_bias_shadow: f32,
    air_ior: f32,
    tree: BVH<Object>,
    pub sky_color: SkyColor,
}

impl Scene {
    pub fn new() -> Scene {
        Scene {
            camera: Camera::new_empty(),
            planes: Vec::new(),
            materials: Vec::new(),
            shaders: Vec::new(),
            textures: Vec::new(),
            lights: Vec::new(),
            light_ambient: Vector3::zeros(),
            default_material_id: 0,
            correction_bias: 0.,
            correction_bias_shadow: 0.01,
            air_ior: 1.,
            tree: BVH::default(),
            objects: Vec::new(),
            sky_color: SkyColor::Cosinus,
        }
    }

    pub fn from_config(file: scene_config::SceneConfig, default_material: &Path) -> Result<Scene, Box<dyn Error>> {
        let mut scene = Scene::new();

        scene.load_mtl(default_material)?[0];

        for obj in file.meshes {
            scene.load_mesh(obj, None)?;
        }

        for sphere in file.spheres {
            scene.load_sphere(sphere.center, sphere.radius, sphere.material)?;
        }

        for volume in file.volumes {
            scene.load_volume(volume)?;
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

    fn get_next_material_id(&self) -> usize { self.objects.len() }

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

    pub fn load_mesh(&mut self, conf: MeshConfig, shader: Option<usize>) -> Result<(), Box<dyn Error>> {
        let r = UnitQuaternion::from_euler_angles(conf.rotation.x, conf.rotation.y, conf.rotation.z);
        self.load_obj(Path::new(&conf.path), conf.position, r, conf.scale, conf.override_material, shader)
    }

    pub fn load_particles(&mut self, path: String, material: Option<String>) -> Result<(), Box<dyn Error>>  {
        let material_id = if let Some(material_path) = material {
            self.load_mtl(Path::new(&material_path))?[0]
        } else { self.default_material_id };

        let particles = Particles::load(&Path::new(&path))?;

        let (positions, radii) = (particles.get_positions(), particles.get_radii());

        for i in 0..particles.len() {
            self.objects.push(
                Box::new(
                    Sphere::new(positions[i], radii[i], material_id, None, self.get_next_material_id())
                )
            );
        }

        Ok(())
    }
    pub fn load_sphere(&mut self, center: Vector3<f32>, radius: f32, material: Option<String>) -> Result<(), Box<dyn Error>> {
        let material_id = if let Some(material_path) = material {
            self.load_mtl(Path::new(&material_path))?[0]
        } else { self.default_material_id };

        self.objects.push(
            Box::new(
                Sphere::new(center, radius, material_id, None, self.get_next_material_id())
            )
        );

        Ok(())
    }

    pub fn load_plane(&mut self, config: &scene_config::PlaneConfig) -> Result<(), Box<dyn Error>> {
        let material_id = if let Some(material_path) = &config.material {
            self.load_mtl(Path::new(&material_path))?[0]
        } else {
            self.default_material_id
        };

        self.planes.push(
            Plane::new(config.axis, config.position, material_id, 0)
        );

        Ok(())
    }

    pub fn load_obj(&mut self, path: &Path, position: Vector3<f32>, rotation: UnitQuaternion<f32>, 
        scale: Vector3<f32>, 
        override_material: Option<String>, 
        shader: Option<usize>) -> Result<(), Box<dyn Error>> {

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

                self.objects.push(
                    Box::new(shapes::Triangle::new(vertices[a],
                                                   vertices[b],
                                                   vertices[c],
                                                   *vertices_normal.get(a).unwrap_or(&normal),
                                                   *vertices_normal.get(b).unwrap_or(&normal),
                                                   *vertices_normal.get(c).unwrap_or(&normal),
                                                   vertices_tex[a],
                                                   vertices_tex[b] - vertices_tex[a],
                                                   vertices_tex[c] - vertices_tex[a],
                                                   mat_id,
                                                   shader,
                                                   self.get_next_material_id(),
                    )));
            }
        }

        Ok(())
    }

    pub fn load_volume(&mut self, config: VolumeConfig) -> Result<(), Box<dyn Error>> {
        if let Some(bubble) = config.bubble {
            self.load_particles(bubble.path, bubble.material)?;
        }

        let mut foam_shader = None;

        if let Some(foam) = config.foam {
            let color = foam.color;
            let particles = Particles::load(&Path::new(&foam.path))?;

            let shader = FoamFragmentShader::new(particles, color, foam.radius, foam.density_scaling_factor);

            foam_shader = Some(self.shaders.len());

            self.shaders.push(Box::new(
                shader
            ));
        }

        self.load_mesh(config.mesh, foam_shader)
    }

    pub fn add_triangles(&mut self, triangles: &Vec<shapes::Triangle>) {
        for v in triangles {
            self.objects.push(Box::new(*v));
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
        }, &self.objects)
    }

    fn sky_color(&self, ray: Ray) -> Vector3<f32> {
        self.sky_color.color(&ray)
    }

    fn compute_diffuse_and_specular(&self, position: &Vector3<f32>, normal: &Vector3<f32>, shininess: f32, mut id_triangle_from: usize) -> (Vector3<f32>, Vector3<f32>) {
        let mut diffuse = Vector3::zeros();
        let mut specular = Vector3::zeros();
        self.lights.iter().filter(|l| match l.shadow_ray(*position) {
            Some(mut v) => {
                let mut has_interference = false;
                while let Some((intersection, triangle)) = self.launch_ray(&v, self.correction_bias_shadow, id_triangle_from) {
                    let material = &self.materials[triangle.material()];
                    has_interference = material.illumination_model < 5; // hack: we just ignore transparent objects
                    if has_interference {
                        break;
                    }
                    let intersection_position = intersection.position(&v);
                    v = Ray::new(intersection_position, v.direction);
                    id_triangle_from = triangle.id();
                }
                !has_interference
            },
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

    fn launch_ray(&self, ray: &Ray, bias: f32, ignore: usize) -> Option<(Intersection, Object)> {
        let mut intersection = self.tree.ray_intersect_with_predicate(ray, bias, &|t| t.id() != ignore).get(0);
        let mut min_dist = if let Some((i, _)) = &intersection {
            i.distance
        } else {
            std::f32::INFINITY
        };

        for plane in &self.planes {
            if let Some(i) = plane.intersects(ray, bias) {
                if i.distance < min_dist {
                    min_dist = i.distance;
                    intersection = Some((i, Box::new(plane.clone())));
                }
            }
        }

        intersection
    }

    fn cast_ray(&self, ray: Ray, max_rec: u8, mut distance_inside_medium: f32, id_triangle_from: usize) -> Vector3<f32> {
        if let Some((i, triangle)) = self.launch_ray(&ray, self.correction_bias, id_triangle_from) {
            let normal = triangle.smoothed_normal(&ray, &i);

            let material = &self.materials[triangle.material()];

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
                let reflect_ray = Ray::new(position, reflect_direction);

                if max_rec == 0 { self.sky_color(reflect_ray) } else {
                    self.cast_ray(reflect_ray, max_rec - 1, distance_inside_medium, triangle.id())
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
                let refract_ray = Ray::new(position, refract_direction);
                let refract_color = if max_rec == 0 {
                    self.sky_color(refract_ray)
                } else {
                    self.cast_ray(refract_ray, max_rec - 1, 0.0, triangle.id())
                };

                if material.illumination_model == 6 {
                    color += (Vector3::new(1.0, 1.0, 1.0) - ks).component_mul(&refract_color);
                } else {
                    color += (1.0 - coeff_reflectivity) * refract_color;
                }
            }

            color += ka.component_mul(&self.light_ambient) + kd.component_mul(&diffuse) + ks.component_mul(&specular);

            color = clamp_light(&color.component_mul(&beer));

            if let Some(shader) = triangle.shader() {
                color = self.shaders[shader].apply(&position, &color);
            }

            color
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
