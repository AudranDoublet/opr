/*
 * Kernel proposed by Akinci & Al in
 * `Surface Tension and Adhesion for SPH Fluids` (2013) for surface tension forces
 */

use crate::kernels::Kernel;

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct CohesionKernel {
    h: f32,
    h2: f32,
    normalizer: f32,
    sigma: f32,
    coeff: f32,
}

impl CohesionKernel {
    pub fn new(smoothing_length: f32) -> CohesionKernel {
        CohesionKernel {
            h: smoothing_length,
            h2: smoothing_length.powi(2),
            normalizer: smoothing_length.powi(-9),
            sigma: 32. / std::f32::consts::PI,
            coeff: smoothing_length.powi(6) / 64.,
        }
    }
}

impl Kernel for CohesionKernel {
    fn apply_on_norm(&self, r_norm: f32) -> f32 {
        self.unormalized_apply_on_norm(r_norm, self.normalizer)
    }

    fn gradient(&self, _r: &Vector3<f32>) -> Vector3<f32> {
        panic!("cohesion kernel: gradient: not implement")
    }

    fn unormalized_apply_on_norm(&self, r_norm: f32, normalizer: f32) -> f32 {
        let q: f32 = r_norm / self.h;

        self.sigma * normalizer * match q {
            q if q <= 0.5 => (self.h - r_norm).powi(3) * r_norm.powi(3) * 2. - self.coeff,
            q if q <= 1.0 => (self.h - r_norm).powi(3) * r_norm.powi(3),
            _ => 0.0
        }
    }

    fn radius(&self) -> f32 {
        self.h
    }

    fn radius_sq(&self) -> f32 {
        self.h2
    }
}
