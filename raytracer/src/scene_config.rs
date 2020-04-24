extern crate serde_yaml;

use crate::*;
use serde_derive::*;
use nalgebra::Vector3;

#[derive(Debug, Deserialize, Serialize)]
pub struct ObjectConfig
{
    pub path: String,
    pub rotation: Vector3<f32>,
    pub position: Vector3<f32>,
    pub scale: Vector3<f32>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct ParamsConfig
{
    #[serde(default)]
    pub build_max_depth: u32,
}

impl Default for ParamsConfig {
    fn default() -> Self {
        ParamsConfig {
            build_max_depth: 13,
        }
    }
}

#[derive(Debug, Deserialize, Serialize)]
pub struct SceneConfig
{
    pub params: ParamsConfig,
    pub objects: Vec<ObjectConfig>,
    pub lights: Vec<Light>,
    pub camera: Camera,
}

impl SceneConfig {
    pub fn save(&self, path: &std::path::Path) -> Result<(), Box<dyn std::error::Error>> {
        serde_yaml::to_writer(std::fs::File::create(path)?, self)?;
        Ok(())
    }

    pub fn load(path: &std::path::Path) -> Result<SceneConfig, Box<dyn std::error::Error>> {
        let file: SceneConfig = serde_yaml::from_reader(std::fs::File::open(path)?)?;
        Ok(file)
    }
}
