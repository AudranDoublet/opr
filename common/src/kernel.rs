extern crate nalgebra;

use nalgebra::Vector3;

pub trait Kernel {
    fn apply_on_norm(r_norm: f32) -> f32;
    fn gradient(r: &Vector3<f32>) -> Vector3<f32>;
    fn radius() -> f32;
    fn radius_sq() -> f32;
}

pub mod kernels {
    use crate::kernel::{Kernel, SMOOTHING_LENGTH, SMOOTHING_LENGTH_INV, SMOOTHING_LENGTH_SQ};

    use super::nalgebra::Vector3;

    pub struct CubicSpine {
        sigma: f32,
        h: f32,
        h2: f32,
        h3: f32,
    }

    impl CubicSpine {
        fn new(smoothing_length: f32) -> CubicSpine {
            CubicSpine {
                sigma: 8. / (std::f32::consts::PI * SMOOTHING_LENGTH.powi(3)),
                h: smoothing_length, 
                h2: smoothing_length.powi(2),
                h3: smoothing_length.powi(3),
            }
        }
    }

    impl Kernel for CubicSpine {

        fn apply_on_norm(r_norm: f32) -> f32 {
            let q: f32 = r_norm / SMOOTHING_LENGTH;

            self.sigma * match q {
                q if q <= 0.5 => 6. * (q.powi(3) - q.powi(2)) + 1.,
                q if q <= 1.0 => 2. * (1. - q).powi(3),
                _ => 0.0
            }
        }

        fn gradient(r: &Vector3<f32>) -> Vector3<f32> {
            let r_norm = r.norm();
            let q = r_norm / self.h;

            self.sigma * r * match q {
                q if q <= 0.0001 || q > 1.0 => 0.0,
                q if q <= 0.5 => 6. * (3. * r_norm / self.h3 - 2. / self.h2),
                q => - 6. / self.h * (1. - q).powi(3),
            }
        }

        fn radius() -> f32 {
            SMOOTHING_LENGTH
        }

        fn radius_sq() -> f32 {
            SMOOTHING_LENGTH_SQ
        }
    }
}
