extern crate serde_yaml;

use nalgebra::Vector3;
use serde_derive::*;

use search::Ray;
use crate::*;

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type")]
pub enum SkyColor {
    #[serde(rename = "cosinus")]
    Cosinus,
    #[serde(rename = "color")]
    Color {
        color: Vector3<f32>,
    }
}

impl Default for SkyColor {
    fn default() -> SkyColor {
        SkyColor::Cosinus
    }
}

impl SkyColor {
    pub fn color(&self, ray: &Ray) -> Vector3<f32> {
        match self {
            SkyColor::Cosinus => ray.direction.apply_into(|f| f.cos()),
            SkyColor::Color { color } => *color,
        }
    }
}

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
pub struct BubbleConfig
{
    pub path: String,
    #[serde(default)]
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
pub struct FoamConfig {
    pub path: String,
    #[serde(default = "default_foam_config_radius")]
    pub radius: f32,
    #[serde(default = "default_foam_config_color")]
    pub color: Vector3<f32>,
    #[serde(default = "default_foam_config_density_scaling_factor")]
    pub density_scaling_factor: f32,
}

fn default_foam_config_radius() -> f32 { 0.5 }
fn default_foam_config_color() -> Vector3<f32> { Vector3::new(1., 1., 1.) }
fn default_foam_config_density_scaling_factor() -> f32 { 1.0 }

#[derive(Debug, Deserialize, Serialize)]
pub struct VolumeConfig {
    pub mesh: MeshConfig,
    #[serde(default)]
    pub bubble: Option<BubbleConfig>,
    #[serde(default)]
    pub foam: Option<FoamConfig>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct SceneConfig
{
    pub spheres: Vec<SphereConfig>,
    pub meshes: Vec<MeshConfig>,
    pub volumes: Vec<VolumeConfig>,
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
