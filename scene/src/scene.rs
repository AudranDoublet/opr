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

#[derive(Debug, Deserialize)]
pub struct CommandLineConfiguration
{
    pub data_path: String,
    pub cache_path: String,
    pub use_cache: bool,
}

#[derive(Debug, Deserialize)]
pub struct Scene
{
    #[serde(skip_serializing)]
    pub global_config: CommandLineConfiguration,
    pub config: Configuration,
    pub solids: Vec<Solid>,
}

impl Scene
{
    pub fn load_solids(&self) -> Vec<RigidObject>
    {
        let mut solids = Vec::new();

        for solid in &self.solids
        {
            solids.push(solid.load(&self));
        }

        solids
    }
}
