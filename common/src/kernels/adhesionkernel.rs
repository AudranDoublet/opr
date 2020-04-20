/*
 * Kernel proposed by Akinci & Al in
 * `Surface Tension and Adhesion for SPH Fluids` (2013) for surface adhesion forces
 */

use crate::kernels::Kernel;

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct AdhesionKernel {
    h: f32,
    h2: f32,
    normalizer: f32,
    sigma: f32,
}

impl AdhesionKernel {
    pub fn new(smoothing_length: f32) -> AdhesionKernel {
        AdhesionKernel {
            h: smoothing_length,
            h2: smoothing_length.powi(2),
            normalizer: smoothing_length.powf(-3.25),
            sigma: 0.007, // harcoded in paper :|
        }
    }
}

impl Kernel for AdhesionKernel {
    fn apply_on_norm(&self, r_norm: f32) -> f32 {
        self.unormalized_apply_on_norm(r_norm, self.normalizer)
    }

    fn gradient(&self, r: &Vector3<f32>) -> Vector3<f32> {
        self.unormalized_gradient(r, self.normalizer)
    }

    fn unormalized_apply_on_norm(&self, r_norm: f32, normalizer: f32) -> f32 {
        let q: f32 = r_norm / self.h;

        self.sigma * normalizer * match q {
            q if q <= 0.5 => 0.0,
            q if q <= 1.0 => (-4.*r_norm*r_norm/self.h + 6.*r_norm - 2.*self.h).powf(0.25),
            _ => 0.0
        }
    }

    fn unormalized_gradient(&self, _r: &Vector3<f32>, _normalizer: f32) -> Vector3<f32> {
        panic!("cohesion kernel: gradient: not implement")
    }

    fn radius(&self) -> f32 {
        self.h
    }

    fn radius_sq(&self) -> f32 {
        self.h2
    }
}
