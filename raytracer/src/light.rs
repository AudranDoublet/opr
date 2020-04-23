use nalgebra::Vector3;
use search::Ray;
use crate::vector3_from_const;
use serde_derive::*;

#[derive(Debug, Deserialize, Clone)]
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
        Light::AmbientLight {
            color: color,
        }
    }

    pub fn directional(direction: Vector3<f32>, color: Vector3<f32>) -> Light {
        Light::DirectionalLight {
            direction: -direction.normalize(),
            color: color,
        }
    }

    pub fn init(&mut self) {
        match *self {
            Light::DirectionalLight { ref mut direction, color: _ } => {
                *direction = -direction.normalize();
            },
            _ => (),
        }
    }

    pub fn apply_light(&self, normal: Vector3<f32>) -> Vector3<f32> {
        match *self {
            Light::AmbientLight { color } => color,
            Light::DirectionalLight { direction, color } => match normal.dot(&direction) {
                                                                v if (v >= 0.0) => v * color,
                                                                _ => vector3_from_const(0.0),
                                                            }
        }
    }

    pub fn shadow_ray(&self, origin: Vector3<f32>) -> Option<Ray> {
        match *self {
            Light::DirectionalLight { direction, color: _ } =>  Some(Ray::new(origin, direction)),
            _ => None
        }
    }
}
