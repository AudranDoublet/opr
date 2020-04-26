extern crate serde_yaml;

use nalgebra::Vector3;

use std::fs::File;
use std::path::Path;

use sph_common::{Emitter, Animation, RigidObject, DFSPH, external_forces::ExternalForces, external_forces::ViscosityType, external_forces::VorticityConfig};

use serde_derive::*;
use crate::{Solid, LiquidZone, EmitterConfig};
use bubbler::config::BubblerConfig;

fn default_gravity() -> [f32; 3] {
    [0.0, -9.81, 0.0]
}

fn default_surface_tension() -> f32 {
    0.05
}

fn default_surface_adhesion() -> f32 {
    0.01
}

#[derive(Debug, Deserialize)]
pub struct CameraConfiguration
{
    pub position: Vector3<f32>,
    pub animation: Animation,
}

impl Default for CameraConfiguration {
    fn default() -> CameraConfiguration {
        CameraConfiguration {
            position: Vector3::zeros(),
            animation: Animation::Blank,
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct Configuration
{
    #[serde(default = "default_gravity")]
    pub gravity: [f32; 3],
    pub kernel_radius: f32, //FIXME mark as default ?
    pub particle_radius: f32,
    #[serde(default = "default_surface_tension")]
    pub surface_tension: f32,
    #[serde(default = "default_surface_adhesion")]
    pub surface_adhesion: f32,
    #[serde(default)]
    pub viscosity: ViscosityType,
    #[serde(default)]
    pub vorticity: VorticityConfig,
}

#[derive(Debug, Deserialize)]
pub struct SimulationConfig
{
    pub max_time: f32,
    pub fps: f32,
    pub render_fps: f32,
}

impl Default for SimulationConfig {
    fn default() -> SimulationConfig {
        SimulationConfig {
            max_time: 4.,
            fps: 24.,
            render_fps: -1.,
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct MeshingConfig
{
    pub iso_value: f32,
    pub cube_size: f32,
    pub enable_interpolation: bool,
    pub enable_anisotropication: bool,
}

impl Default for MeshingConfig {
    fn default() -> MeshingConfig {
        MeshingConfig {
            iso_value: 0.05,
            cube_size: 0.04,
            enable_interpolation: true,
            enable_anisotropication: true,
        }
    }
}

#[derive(Debug)]
pub struct CommandLineConfiguration
{
    pub data_path: String,
    pub cache_path: String,
    pub use_cache: bool,
}

impl Default for CommandLineConfiguration {
    fn default() -> CommandLineConfiguration {
        CommandLineConfiguration {
            data_path: "data/".to_string(),
            cache_path: "cache/".to_string(),
            use_cache: true,
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct Scene
{
    #[serde(skip_deserializing)]
    pub global_config: CommandLineConfiguration,
    #[serde(default)]
    pub simulation_config: SimulationConfig,
    #[serde(default)]
    pub meshing_config: MeshingConfig,
    #[serde(default)]
    pub camera: CameraConfiguration,
    #[serde(default)]
    pub bubbler_config: BubblerConfig,
    pub config: Configuration,
    pub solids: Vec<Solid>,
    pub liquids_blocks: Vec<LiquidZone>,
    pub liquids_add_blocks: Vec<LiquidZone>,
    #[serde(default)]
    pub emitters: Vec<EmitterConfig>,
}

impl Scene
{
    pub fn create_cache_dir(&self) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::create_dir_all(Path::new(&self.global_config.cache_path))?;
        Ok(())
    }

    pub fn gravity(&self) -> Vector3<f32> {
        let g = &self.config.gravity;
        Vector3::new(g[0], g[1], g[2])
    }


    pub fn load_solids(&self) -> Result<Vec<RigidObject>, Box<dyn std::error::Error>>
    {
        self.create_cache_dir()?;

        let mut solids = Vec::new();

        for solid in &self.solids
        {
            solids.push(solid.load(&self)?);
        }

        Ok(solids)
    }

    pub fn load(&self) -> Result<DFSPH, Box<dyn std::error::Error>> {
        self.create_cache_dir()?;

        let solids = self.load_solids()?;
        let mut forces = ExternalForces::new();

        forces.gravity(self.gravity())
              .surface_tension(self.config.kernel_radius, self.config.surface_tension, self.config.surface_adhesion)
              .viscosity(&self.config.viscosity)
              .vorticity(&self.config.vorticity);

        let (emitters, emitters_animations) = self.emitters();

        let mut result = DFSPH::new(
            self.config.kernel_radius,
            self.config.particle_radius,
            solids,
            forces,
            self.camera.position,
            self.camera.animation.clone(),
            emitters,
            emitters_animations,
        );

        self.recreate(&mut result)?;

        Ok(result)
    }

    pub fn emitters(&self) -> (Vec<Emitter>, Vec<Animation>) {
        self.emitters
            .iter()
            .map(|v| v.load(self))
            .unzip()
    }

    pub fn recreate(&self, scene: &mut DFSPH) -> Result<(), Box<dyn std::error::Error>> {
        scene.clear();

        for liquid in &self.liquids_blocks {
            liquid.create_particles(self, scene)?;
        }

        scene.sync();

        Ok(())
    }

    pub fn add_blocks(&self, scene: &mut DFSPH) -> Result<std::ops::Range<usize>, Box<dyn std::error::Error>> {
        let prev_size = scene.len();

        for liquid in &self.liquids_add_blocks {
            liquid.create_particles(self, scene)?;
        }

        scene.sync();

        Ok(prev_size..scene.len())
    }
}

pub fn load_scene(name: &str) -> Result<Scene, Box<dyn std::error::Error>> {
    let result: Scene = serde_yaml::from_reader(&File::open(name)?)?;
    result.bubbler_config.assert_valid();

    Ok(result)
}
