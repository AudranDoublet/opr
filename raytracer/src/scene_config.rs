extern crate serde_yaml;

use nalgebra::Vector3;
use serde_derive::*;

use crate::*;

#[derive(Debug, Deserialize, Serialize)]
pub struct MeshConfig
{
    pub path: String,
    pub override_material: Option<String>,
    pub rotation: Vector3<f32>,
    pub position: Vector3<f32>,
    pub scale: Vector3<f32>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct ParticleConfig
{
    pub path: String,
    pub material: Option<String>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct SphereConfig
{
    pub material: Option<String>,
    pub center: Vector3<f32>,
    pub radius: f32,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct SceneConfig
{
    pub spheres: Vec<SphereConfig>,
    pub meshes: Vec<MeshConfig>,
    pub particles: Vec<ParticleConfig>,
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
