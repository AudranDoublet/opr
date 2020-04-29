#![feature(clamp)]

pub mod kernels;
pub mod mesh;

mod gausslegendre;
mod conjugate_gradient_solver;
mod curves;
mod discretegrid;

pub use gausslegendre::*;
pub use conjugate_gradient_solver::*;
pub use curves::*;
pub use discretegrid::*;

use nalgebra::Vector3;

pub fn orthogonal_vectors(v: Vector3<f32>) -> Vec<Vector3<f32>> {
    let mut up = Vector3::x();

    if up.dot(&v) > 0.999 {
        up = Vector3::y();
    }

    let a = v.cross(&up);
    let b = v.cross(&a);

    vec![a, -a, b, -b]
}
