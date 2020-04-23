extern crate tobj;
extern crate rayon;
extern crate serde_yaml;

use search::{BVH, Ray, BVHParameters};
use nalgebra::{Vector3, Vector2};
use std::collections::HashMap;
use std::error::Error;
use std::path::Path;
use std::fs::File;

use rayon::prelude::*;

use crate::*;

pub struct Scene
{
    camera: Camera,
    triangles: Vec<shapes::Triangle>,
    materials: Vec<Material>,
    textures: Vec<Texture>,
    lights: Vec<Light>,

    tree: Option<BVH<shapes::Triangle>>,
}

fn load_texture(root: &Path, path: &Path) -> Result<Texture, Box<dyn Error>>
{
    let (width, height, pixels) = load_image(&root.join(path))?;
    let tex = Texture::new(width, height, pixels);

    Ok(tex)
}

impl Scene {
    pub fn new() -> Scene {
        Scene {
            camera: Camera::new_empty(),
            triangles: Vec::new(),
            materials: Vec::new(),
            textures: Vec::new(),
            lights: Vec::new(),
            tree: None,
        }
    }

    pub fn from_config(file: scene_config::SceneConfig, default_material: &Path) -> Result<Scene, Box<dyn Error>> {
        let mut scene = Scene::new();

        scene.load_mtl(default_material)?[0];

        for obj in file.objects {
            scene.load_obj(Path::new(&obj.path))?;
        }

        for mut light in file.lights {
            light.init();
            scene.add_light(light);
        }

        scene.camera = file.camera;

        Ok(scene)
    }

    pub fn from_file(path: &Path, default_material: &Path) -> Result<Scene, Box<dyn Error>> {
        let file : scene_config::SceneConfig = serde_yaml::from_reader(File::open(path)?)?;

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

    pub fn load_obj(&mut self, path: &Path) -> Result<(), Box<dyn Error>> {
        let root = path.parent().ok_or("bad path")?;
        let (models, materials) = tobj::load_obj(path)?;

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
            let mat_id = if let Some(mat) = mesh.material_id {
                mat + mat_id_app
            } else {
                0
            };

            let mut vertices = vec![];

            for i in (0..mesh.positions.len()).step_by(3)
            {
                vertices.push(Vector3::new(mesh.positions[i + 0],
                                           mesh.positions[i + 1],
                                           mesh.positions[i + 2]));
            }

            let mut vertices_normal = vec![];

            for i in (0..mesh.normals.len()).step_by(3)
            {
                vertices_normal.push(Vector3::new(mesh.normals[i + 0],
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

                self.triangles.push(
                    shapes::Triangle::new(vertices[a],
                                          vertices[b],
                                          vertices[c],
                                          vertices_normal[a],
                                          vertices_normal[b],
                                          vertices_normal[c],
                                          vertices_tex[a],
                                          vertices_tex[b] - vertices_tex[a],
                                          vertices_tex[c] - vertices_tex[a],
                                          mat_id,
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
        self.tree = Some(BVH::build_params(&BVHParameters {
            max_depth: max_depth as usize,
            max_shapes_in_leaf: 1,
            bucket_count: 6,
        }, &self.triangles))
    }

    fn clamp_light(&self, v: Vector3<f32>) -> Vector3<f32> {
        Vector3::new(
            v.x.min(1.0).max(0.0),
            v.y.min(1.0).max(0.0),
            v.z.min(1.0).max(0.0),
        )
    }

    fn cast_ray(&self, ray: Ray, max_rec: u32, apply_lights: bool) -> Option<Vector3<f32>> {
        if let Some(tree) = &self.tree {
            if let Some((i, triangle)) = tree.ray_intersect(&ray) {
                let normal = (triangle.v1_normal * (1.0 - i.u - i.v)
                          + triangle.v2_normal * i.u
                          + triangle.v3_normal * i.v).normalize();

                let position = i.position(&ray);
                let material = &self.materials[triangle.material];

                let reflected_ray = ray.direction - 2.0 * (normal.dot(&ray.direction)) * normal;

                let mut triangle_color = material.get_diffuse(&self.textures, &triangle, i.u, i.v);

                if apply_lights {
                    triangle_color = triangle_color.component_mul(
                        &self.clamp_light(
                            self.lights
                            .iter()
                            .filter(|l| match l.shadow_ray(position) {
                                Some(v) => !self.cast_ray(v, 0, false).is_some(),
                                _ => true
                            })
                            .map(|l| l.apply_light(normal))
                            .fold(Vector3::zeros(), |a, b| a + b)
                        )
                    );
                }

                Some(triangle_color)
            } else {
                None
            }
        } else {
            None
        }
    }

    pub fn render(&mut self, width: usize, height: usize) -> Vec<Vector3<f32>> {
        self.camera.set_size(width as f32, height as f32);

        (0..width * height).into_par_iter()
                           .map(|i| self.cast_ray(self.camera.generate_ray((i % width) as f32, (i / width) as f32), 10, true)
                                        .unwrap_or(Vector3::zeros()))
                           .collect()
    }
}
