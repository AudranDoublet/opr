use nalgebra::Vector3;

pub trait Kernel {
    fn apply_on_norm(&self, r_norm: f32) -> f32;
    fn gradient(&self, r: &Vector3<f32>) -> Vector3<f32>;
    fn unormalized_apply_on_norm(&self, r_norm: f32, normalizer: f32) -> f32;

    fn radius(&self) -> f32;
    fn radius_sq(&self) -> f32;
}

mod cubicspine;
mod cohesionkernel;
mod adhesionkernel;

pub use cubicspine::*;
pub use cohesionkernel::*;
pub use adhesionkernel::*;
