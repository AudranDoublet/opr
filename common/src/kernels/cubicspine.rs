use crate::kernels::Kernel;

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct CubicSpine {
    sigma: f32,
    normalizer: f32,
    h: f32,
    h2: f32,
}

impl CubicSpine {
    pub fn new(smoothing_length: f32) -> CubicSpine {
        CubicSpine {
            sigma: 8. / std::f32::consts::PI,
            normalizer: smoothing_length.powi(-3),
            h: smoothing_length,
            h2: smoothing_length.powi(2),
        }
    }
}

impl Kernel for CubicSpine {
    fn apply_on_norm(&self, r_norm: f32) -> f32 {
        self.unormalized_apply_on_norm(r_norm, self.normalizer)
    }

    fn gradient(&self, r: &Vector3<f32>) -> Vector3<f32> {
        self.unormalized_gradient(r, self.normalizer)
    }

    fn unormalized_apply_on_norm(&self, r_norm: f32, normalizer: f32) -> f32 {
        let q: f32 = r_norm / self.h;

        self.sigma * normalizer * match q {
            q if q <= 0.5 => 6. * (q.powi(3) - q.powi(2)) + 1.,
            q if q <= 1.0 => 2. * (1. - q).powi(3),
            _ => 0.0
        }
    }

    fn unormalized_gradient(&self, r: &Vector3<f32>, normalizer: f32) -> Vector3<f32> {
        let r_norm = r.norm();
        let q = r_norm / self.h;

        6. * self.sigma * normalizer * r * (1. / (r_norm * self.h)) * match q {
            q if r_norm <= 1e-5 || q > 1.0 => 0.0,
            q if q <= 0.5 => q * (3. * q - 2.),
            q => -(1. - q).powi(2),
        }
    }

    fn radius(&self) -> f32 {
        self.h
    }

    fn radius_sq(&self) -> f32 {
        self.h2
    }
}
