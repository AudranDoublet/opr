use nalgebra::Vector3;
use search::Ray;
use serde_derive::*;

#[derive(Debug, Deserialize, Serialize, Clone)]
#[serde(tag = "type")]
pub enum Light {
    #[serde(rename = "ambient")]
    AmbientLight { color: Vector3<f32> },
    #[serde(rename = "directional")]
    DirectionalLight { color: Vector3<f32>, direction: Vector3<f32> },
}

impl Light
{
    pub fn ambient(color: Vector3<f32>) -> Light {
        Light::AmbientLight { color }
    }

    pub fn directional(direction: Vector3<f32>, color: Vector3<f32>) -> Light {
        Light::DirectionalLight {
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

    pub fn get_direction(&self, _target: &Vector3<f32>) -> Vector3<f32> {
        match *self {
            Light::AmbientLight { .. } => Vector3::new(1., 1., 1.),
            Light::DirectionalLight { direction, .. } => direction
        }
    }
    pub fn get_color(&self) -> Vector3<f32> {
        match *self {
            Light::AmbientLight { color, .. } => color,
            Light::DirectionalLight { color, .. } => color
        }
    }

    pub fn shadow_ray(&self, origin: Vector3<f32>) -> Option<Ray> {
        match *self {
            Light::DirectionalLight { direction, .. } => Some(Ray::new(origin, direction)),
            _ => None
        }
    }
}
