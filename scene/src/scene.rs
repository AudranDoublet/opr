extern crate serde_yaml;

use std::collections::HashMap;
use std::fs::File;
use std::path::Path;

use nalgebra::Vector3;
use serde::Deserialize;
use sph::{Animation, Emitter, Fluid, RigidObject, Simulation};

use crate::{EmitterConfig, FluidConfiguration, LiquidZone, Solid};

use raytracer::{Light, scene_config::SkyColor};

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
    pub particle_radius: f32,
}

#[derive(Debug, Deserialize)]
pub struct SimulationConfig
{
    pub max_time: f32,
    pub fps: f32,
    #[serde(default)]
    pub enable_bubbler: bool,
}

impl Default for SimulationConfig {
    fn default() -> SimulationConfig {
        SimulationConfig {
            max_time: 4.,
            fps: 24.,
            enable_bubbler: false,
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct RenderConfig {
    #[serde(default = "render_conf_default_fps")]
    pub fps: f32,
    #[serde(default = "render_conf_default_resolution")]
    pub resolution: (usize, usize),
    #[serde(default = "render_conf_default_build_max_def")]
    pub build_max_depth: u32,
    #[serde(default = "render_conf_default_max_rec")]
    pub max_rec: u8,
    #[serde(default = "render_conf_default_aa_max_sample")]
    pub aa_max_sample: usize,
    #[serde(default = "render_conf_default_lights")]
    pub lights: Vec<Light>,
    #[serde(default)]
    pub sky_color: SkyColor,
}

fn render_conf_default_fps() -> f32 { -1. }

fn render_conf_default_resolution() -> (usize, usize) { (512, 512) }

fn render_conf_default_build_max_def() -> u32 { 12 }

fn render_conf_default_max_rec() -> u8 { 10 }

fn render_conf_default_aa_max_sample() -> usize { 16 }

fn render_conf_default_lights() -> Vec<Light> {
    vec![
        Light::ambient(Vector3::new(0.3, 0.3, 0.3)),
        Light::directional(Vector3::new(-1., 1., -1.), Vector3::new(0.5, 0.5, 0.5)),
    ]
}

impl Default for RenderConfig {
    fn default() -> RenderConfig {
        RenderConfig {
            fps: render_conf_default_fps(),
            resolution: render_conf_default_resolution(),
            build_max_depth: render_conf_default_build_max_def(),
            max_rec: render_conf_default_max_rec(),
            aa_max_sample: render_conf_default_aa_max_sample(),
            lights: render_conf_default_lights(),
            sky_color: SkyColor::default(),
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
    pub render_config: RenderConfig,
    #[serde(default)]
    pub camera: CameraConfiguration,
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

    pub fn kernel_radius(&self) -> f32 {
        4. * self.config.particle_radius
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

        let map = self.load_fluids_map();

        for (k, &idx) in map.iter() {
            fluids.push(self.fluids[k].create(idx, self.volume(), self.gravity(), self.kernel_radius()));
        }

        fluids
    }

    pub fn load_fluids_map(&self) -> HashMap<String, usize> {
        let mut names = self.fluids.iter()
                       .map(|(k, _)| k).collect::<Vec<&String>>();

        names.sort();
        names.iter()
             .enumerate()
             .map(|(i, k)| ((*k).clone(), i))
             .collect()
    }

    pub fn load(&self) -> Result<Simulation, Box<dyn std::error::Error>> {
        self.create_cache_dir()?;

        let solids = self.load_solids()?;
        let fluids = self.load_fluids();

        let (emitters, emitters_animations) = self.emitters();

        let mut result = Simulation::new(
            self.kernel_radius(),
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

    Ok(result)
}
