use nalgebra::Vector3;
use search::Ray;
use serde_derive::*;

use crate::Material;

#[derive(Debug, Deserialize, Serialize, Clone)]
#[serde(tag = "type")]
pub enum Light {
    #[serde(rename = "ambient")]
    AmbientLight { intensity: f32, color: Vector3<f32> },
    #[serde(rename = "directional")]
    DirectionalLight { intensity: f32, color: Vector3<f32>, direction: Vector3<f32> },
}

impl Light
{
    pub fn ambient(intensity: f32, color: Vector3<f32>) -> Light {
        Light::AmbientLight {
            intensity,
            color,
        }
    }

    pub fn directional(intensity: f32, direction: Vector3<f32>, color: Vector3<f32>) -> Light {
        Light::DirectionalLight {
            intensity,
            direction: -direction.normalize(),
            color,
        }
    }

    pub fn init(&mut self) {
        match *self {
            Light::DirectionalLight { ref mut direction, .. } => {
                *direction = -direction.normalize();
            }
            _ => (),
        }
    }

    pub fn apply_light(&self, view_dir: &Vector3<f32>, normal: &Vector3<f32>, reflect: &Vector3<f32>, material: &Material, diffuse: &Vector3<f32>) -> Vector3<f32> {
        match *self {
            Light::AmbientLight { intensity, color } => intensity * color.component_mul(&material.get_ambient()),
            Light::DirectionalLight { intensity, direction, color } => {
                let diffuse = diffuse * normal.dot(&direction).max(0.0);
                let specular = &material.specular * view_dir.dot(&reflect).max(0.0).powf(material.shininess);
                intensity * color.component_mul(&(diffuse + specular))
            }
        }
    }

    pub fn shadow_ray(&self, origin: Vector3<f32>) -> Option<Ray> {
        match *self {
            Light::DirectionalLight { direction, .. } => Some(Ray::new(origin, direction)),
            _ => None
        }
    }
}
