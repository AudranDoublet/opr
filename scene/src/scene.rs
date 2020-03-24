extern crate serde_yaml;

use std::fs::File;
use std::path::Path;

use sph_common::{RigidObject};

use serde_derive::*;
use crate::Solid;

#[derive(Debug, Deserialize)]
pub struct Configuration
{
    #[serde(default)]
    pub gravity: [f32; 3],
    pub kernel_radius: f32,
}

impl Default for Configuration
{
    fn default() -> Self {
        Configuration {
            gravity: [0.0, -9.81, 0.0],
            kernel_radius: 0.0, // FIXME default ?
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
    pub config: Configuration,
    pub solids: Vec<Solid>,
}

impl Scene
{
    pub fn create_cache_dir(&self) -> Result<(), Box<dyn std::error::Error>> {
        std::fs::create_dir_all(Path::new(&self.global_config.cache_path))?;
        Ok(())
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
}

pub fn load_scene(name: &str) -> Result<Scene, Box<dyn std::error::Error>> {
    let result: Scene = serde_yaml::from_reader(&File::open(name)?)?;

    Ok(result)
}
