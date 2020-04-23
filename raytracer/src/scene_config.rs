use crate::*;
use serde_derive::*;
use nalgebra::Vector3;

#[derive(Debug, Deserialize)]
pub struct ObjectConfig
{
    pub path: String,
    pub rotation: Vector3<f32>,
    pub position: Vector3<f32>,
}

#[derive(Debug, Deserialize)]
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

#[derive(Debug, Deserialize)]
pub struct SceneConfig
{
    pub params: ParamsConfig,
    pub objects: Vec<ObjectConfig>,
    pub lights: Vec<Light>,
    pub camera: Camera,
}
