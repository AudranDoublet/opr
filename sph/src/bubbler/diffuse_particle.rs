use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Serialize, Deserialize)]
pub struct DiffuseParticle {
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub lifetime: f32,
    pub kind: DiffuseParticleType,
    pub radius: f32,
    pub power: f32,
}

#[derive(Copy, Clone, Serialize, Deserialize, Eq, PartialEq)]
pub enum DiffuseParticleType {
    Spray,
    Foam,
    Bubble,
}

impl DiffuseParticle {
    pub fn is_dissolved(&self) -> bool { self.lifetime <= 0.0 }
}
