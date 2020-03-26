use sph_common::{DiscreteGrid, RigidObject, mesh::Mesh, kernels::CubicSpine};

use crate::Scene;

use std::path::{Path, PathBuf};

use serde_derive::*;
use nalgebra::Vector3;

use std::fs::File;

#[derive(Debug, Deserialize)]
pub struct Solid {
    pub mesh: String,
    pub mesh_invert: bool,

    pub scale: [f32; 3],
    pub position : [f32; 3],
    pub rotation_axis: [f32; 3],
    pub rotation_angle: f32,

    pub resolution: [u32; 3],
    pub display: bool,
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
        let cache_file = format!("{}_{}_{}_{}_{}_{}.cache",
                                    self.mesh.replace("/", "_"),
                                    self.mesh_invert,
                                    self.resolution[0],
                                    self.resolution[1],
                                    self.resolution[2],
                                    radius,
                         );

        cache_path.join(cache_file)
    }

    pub fn load(&self, scene: &Scene) -> Result<RigidObject, Box<dyn std::error::Error>> {
        let kradius = scene.config.kernel_radius;

        let cache_file = self.cache_file(Path::new(&scene.global_config.cache_path), kradius);
        let mesh_file = self.file(Path::new(&scene.global_config.data_path));

        let grid = if scene.global_config.use_cache && cache_file.as_path().exists() {
            println!("Use cache {:?} for {}", cache_file, self.mesh);

            let file = File::open(&cache_file)?;
            DiscreteGrid::load(&file)?
        } else {
            println!("Compute sdf and volume for {}", self.mesh);
            let extend = 2. * Vector3::new(kradius, kradius, kradius);

            let mut mesh = Mesh::load_obj(&mesh_file)?;

            if self.mesh_invert {
                mesh.invert();
            }

            let mut grid = DiscreteGrid::new(
                mesh.bounding_min - extend,
                mesh.bounding_max + extend,
                self.resolution(),
            );

            mesh.compute_sdf(&mut grid);
            mesh.compute_volume(&mut grid, &CubicSpine::new(kradius));

            println!("Save discretegrid into {:?}", cache_file);
            let file = File::create(&cache_file)?;
            grid.save(&file)?;

            grid
        };

        println!("{:?}", grid.interpolate(1, Vector3::new(0.5, 0.5, 0.5), true));
        println!("{} loaded!", self.mesh);

        let mut object = RigidObject::new(grid);

        object.set_scale(self.scale());
        object.set_position(self.position());

        Ok(object)
    }
}
