use nalgebra::Vector3;

#[derive(Copy, Clone)]
pub struct DiffuseParticle {
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub lifetime: f32,
}

pub enum DiffuseParticleType {
    Spray,
    Foam,
    Bubble,
}

impl DiffuseParticle {
    pub fn is_dissolved(&self) -> bool { self.lifetime <= 0.0 }
}
