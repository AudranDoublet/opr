mod camera;
mod material;
mod scene;
mod light;
mod image;

pub mod scene_config;
pub mod shapes;

pub use camera::*;
pub use material::*;
pub use scene::*;
pub use light::*;
pub use crate::image::*;

use nalgebra::Vector3;

#[inline]
pub fn vector3_from_array(a: &[f32; 3]) -> Vector3<f32> {
    Vector3::new(a[0], a[1], a[2])
}

#[inline]
pub fn vector3_from_const(f: f32) -> Vector3<f32> {
    Vector3::new(f, f, f)
}
