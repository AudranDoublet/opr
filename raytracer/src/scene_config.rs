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
            SkyColor::Color { color } => {
                let coeff = ray.direction.normalize().dot(&Vector3::y()).abs() * 0.2 + 0.8;

                coeff * color
            },
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type")]
pub struct PlaneConfig {
    pub axis: usize,
    pub position: f32,
    #[serde(default)]
    pub material: Option<String>,
}

fn default_center_of_mass() -> Vector3<f32> {
    Vector3::zeros()
}

#[derive(Debug, Deserialize, Serialize)]
pub struct MeshConfig
{
    pub path: String,
    pub override_material: Option<String>,
    pub rotation: Vector3<f32>,
    pub position: Vector3<f32>,
    pub scale: Vector3<f32>,
    #[serde(default = "default_center_of_mass")]
    pub center_of_mass: Vector3<f32>,
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
pub struct VolumeConfig {
    pub mesh: MeshConfig,
    #[serde(default)]
    pub bubble: Option<BubbleConfig>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct SceneConfig
{
    #[serde(default)]
    pub spheres: Vec<SphereConfig>,
    #[serde(default)]
    pub meshes: Vec<MeshConfig>,
    #[serde(default)]
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
