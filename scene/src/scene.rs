extern crate serde_yaml;

use nalgebra::Vector3;

use std::fs::File;
use std::path::Path;
use std::collections::HashMap;

use sph::{Fluid, Emitter, Animation, RigidObject, Simulation};

use serde_derive::*;
use crate::{Solid, LiquidZone, EmitterConfig, FluidConfiguration};
use bubbler::config::BubblerConfig;

fn default_gravity() -> [f32; 3] {
    [0.0, -9.81, 0.0]
}

#[derive(Debug, Deserialize)]
pub struct CameraConfiguration
{
    pub position: Vector3<f32>,
    #[serde(default)]
    pub generate_at_render: bool,
    pub animation: Animation,
}

impl Default for CameraConfiguration {
    fn default() -> CameraConfiguration {
        CameraConfiguration {
            generate_at_render: false,
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
}

#[derive(Debug, Deserialize)]
pub struct SimulationConfig
{
    pub max_time: f32,
    pub fps: f32,
}

impl Default for SimulationConfig {
    fn default() -> SimulationConfig {
        SimulationConfig {
            max_time: 4.,
            fps: 24.,
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct RenderConfig
{
    pub fps: f32,
    pub resolution: (usize, usize),
}

impl Default for RenderConfig {
    fn default() -> RenderConfig {
        RenderConfig {
            fps: -1.,
            resolution: (512, 512),
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
    pub render_config: RenderConfig,
    #[serde(default)]
    pub camera: CameraConfiguration,
    #[serde(default)]
    pub bubbler_config: BubblerConfig,
    pub config: Configuration,
    pub fluids: HashMap<String, FluidConfiguration>,
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

    pub fn volume(&self) -> f32 {
        4. * std::f32::consts::PI * self.config.particle_radius.powi(3) / 3.
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

    pub fn load_fluids(&self) -> Vec<Fluid> {
        let mut fluids = Vec::new();
        let mut idx = 0;

        for (_, v) in self.fluids.iter() {
            fluids.push(v.create(idx, self.volume(), self.gravity(), self.config.kernel_radius));
            idx += 1;
        }

        fluids
    }

    pub fn load_fluids_map(&self) -> HashMap<String, usize> {
        let mut idx = 0;
        let mut map = HashMap::new();

        for (k, _) in self.fluids.iter() {
            map.insert(k.clone(), idx);
            idx += 1;
        }

        map
    }

    pub fn load(&self) -> Result<Simulation, Box<dyn std::error::Error>> {
        self.create_cache_dir()?;

        let solids = self.load_solids()?;
        let fluids = self.load_fluids();

        let (emitters, emitters_animations) = self.emitters();

        let mut result = Simulation::new(
            self.config.kernel_radius,
            self.config.particle_radius,
            solids,
            fluids,
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

    pub fn recreate(&self, scene: &mut Simulation) -> Result<(), Box<dyn std::error::Error>> {
        scene.clear();

        for liquid in &self.liquids_blocks {
            liquid.create_particles(self, scene)?;
        }

        scene.sync();

        Ok(())
    }

    pub fn add_blocks(&self, scene: &mut Simulation) -> Result<std::ops::Range<usize>, Box<dyn std::error::Error>> {
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
