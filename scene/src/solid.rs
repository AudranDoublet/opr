extern crate image;

use utils::{DiscreteGrid, mesh::Mesh};
use sph::{RigidObject, Animation};
use utils::kernels::CubicSpine;

use crate::Scene;

use std::path::{Path, PathBuf};

use serde_derive::*;
use nalgebra::Vector3;

use std::fs::File;

fn particle_size() -> f32 {
    0.01
}

#[derive(Debug, Deserialize)]
pub struct Solid {
    pub mesh: String,
    pub mesh_invert: bool,

    pub scale: [f32; 3],
    pub position : [f32; 3],
    pub rotation_axis: [f32; 3],
    pub rotation_angle: f32,

    pub density: f32,
    pub resolution: [u32; 3],

    #[serde(default)]
    pub material: Option<String>,

    #[serde(default = "particle_size")]
    pub particle_size: f32,

    #[serde(default)]
    pub display: bool,

    #[serde(default)]
    pub dynamic: bool,

    #[serde(default)]
    pub slice: bool,

    #[serde(default)]
    pub animation: Animation,
}

impl Solid {
    pub fn scale(&self) -> Vector3<f32>
    {
        Vector3::new(self.scale[0], self.scale[1], self.scale[2])
    }

    pub fn position(&self) -> Vector3<f32>
    {
        Vector3::new(self.position[0], self.position[1], self.position[2])
    }

    pub fn rotation_axis(&self) -> Vector3<f32>
    {
        Vector3::new(self.rotation_axis[0], self.rotation_axis[1], self.rotation_axis[2])
    }

    pub fn resolution(&self) -> Vector3<u32>
    {
        Vector3::new(self.resolution[0], self.resolution[1], self.resolution[2])
    }

    pub fn file(&self, data_path: &Path) -> PathBuf
    {
        data_path.join(&self.mesh)
    }

    pub fn cache_file(&self, cache_path: &Path, radius: f32) -> PathBuf
    {
        let cache_file = format!("{}_{}_{}_{}_{}_{}_{}_{}_{}.cache",
                                    self.mesh.replace("/", "_"),
                                    self.mesh_invert,
                                    self.resolution[0],
                                    self.resolution[1],
                                    self.resolution[2],
                                    self.scale[0],
                                    self.scale[1],
                                    self.scale[2],
                                    radius,
                         );

        cache_path.join(cache_file)
    }

    pub fn save_slice(&self, (min, max): (&Vector3<f32>, &Vector3<f32>), f: &dyn Fn(Vector3<f32>) -> Option<f32>) -> Result<(), Box<dyn std::error::Error>> {
        let width = 512;
        let height = 512;

        let mut result = Vec::new();

        let step = (max - min).component_div(&Vector3::new(width as f32, height as f32, 1.));

        let scale = |v: f32| (v.min(0.1) * 10. * 255.) as u8;

        for y in 0..height {
            for x in 0..width {
                let pos = min + step.component_mul(&Vector3::new(x as f32, y as f32, 0.4));

                let (r, g, b) = if let Some(d) = f(pos) {
                    if d < 0.0 {
                        (scale(-d), 0, 0)
                    } else {
                        (0, scale(d), 0)
                    }
                } else {
                    (0, 0, 0)
                };

                result.push(r);
                result.push(g);
                result.push(b);
            }
        }

        image::save_buffer(
            &std::path::Path::new("slice.png"), &result, width as u32, height as u32, image::ColorType::Rgb8
        )?;

        Ok(())
    }

    pub fn load(&self, scene: &Scene) -> Result<RigidObject, Box<dyn std::error::Error>> {
        let kradius = scene.kernel_radius();

        let cache_file = self.cache_file(Path::new(&scene.global_config.cache_path), kradius);
        let mesh_file = self.file(Path::new(&scene.global_config.data_path));

        let mut mesh = Mesh::load_obj(&mesh_file, self.scale())?;

        if self.mesh_invert {
            mesh.invert();
        }

        let properties = mesh.compute_mass_properties(self.density);
        mesh.set_translate(properties.center_of_mass);

        let grid = if scene.global_config.use_cache && cache_file.as_path().exists() {
            println!("Use cache {:?} for {}", cache_file, self.mesh);

            let file = File::open(&cache_file)?;
            DiscreteGrid::load(&file)?
        } else {
            println!("Compute sdf and volume for {}", self.mesh);
            let extend = 2. * Vector3::new(kradius, kradius, kradius);

            let (min, max) = mesh.boundings();
            let mut grid = DiscreteGrid::new(
                min - extend,
                max + extend,
                self.resolution(),
            );

            mesh.compute_sdf(&mut grid);
            mesh.compute_volume(&mut grid, &CubicSpine::new(kradius));

            println!("Save discretegrid into {:?}", cache_file);
            let file = File::create(&cache_file)?;
            grid.save(&file)?;

            grid
        };

        if self.slice {
            self.save_slice(grid.get_domain_definition(), &|p| match grid.interpolate(0, p, false) {
                Some((d, _)) => Some(d),
                _ => None
            })?;
        }

        println!("{} loaded!", self.mesh);

        let mut object = RigidObject::new(grid, self.dynamic, self.particle_size, properties);

        object.set_rotation(self.rotation_axis());
        object.set_position(self.position());
        object.set_animation(self.animation.clone());

        println!("obj_rotation: {:?}", object.rotation());

        Ok(object)
    }
}
