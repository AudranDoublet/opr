extern crate nalgebra;

use nalgebra::Vector3;

pub const SMOOTHING_LENGTH: f32 = 1.0;

const SMOOTHING_LENGTH_SQ: f32 = SMOOTHING_LENGTH * SMOOTHING_LENGTH;
const SMOOTHING_LENGTH_INV: f32 = 1.0 / SMOOTHING_LENGTH;

pub trait Kernel {
    fn apply_on_norm(r_norm: f32) -> f32;
    fn gradient(r: &Vector3<f32>) -> Vector3<f32>;
    fn radius() -> f32;
    fn radius_sq() -> f32;
}

pub mod kernels {
    use crate::kernel::{Kernel, SMOOTHING_LENGTH, SMOOTHING_LENGTH_INV, SMOOTHING_LENGTH_SQ};

    use super::nalgebra::Vector3;

    pub struct CubicSpine;


    impl CubicSpine {
        const SIG_3: f32 = 8.0 / (std::f32::consts::PI * SMOOTHING_LENGTH * SMOOTHING_LENGTH * SMOOTHING_LENGTH);
        const SIG_L: f32 = 48.0 / (std::f32::consts::PI * SMOOTHING_LENGTH * SMOOTHING_LENGTH * SMOOTHING_LENGTH);
    }

    impl Kernel for CubicSpine {
        fn apply_on_norm(r_norm: f32) -> f32 {
            let q: f32 = SMOOTHING_LENGTH_INV * r_norm;

            CubicSpine::SIG_3 * match q {
                q if q <= 1.0 => {
                    if q <= 0.5 {
                        let q_2: f32 = q.powi(2);
                        6.0 * (q_2 * q - q_2) + 1.0
                    } else {
                        2.0 * (1.0 - q).powi(3)
                    }
                }
                _ => 0.0
            }
        }

        fn gradient(r: &Vector3<f32>) -> Vector3<f32> {
            let r_norm = r.norm();
            let q = SMOOTHING_LENGTH_INV * r_norm;
            if (r_norm <= 1.0e-5) || (q > 1.0) {
                return Vector3::zeros();
            }
            let gradq = r * (1.0 / (r_norm * SMOOTHING_LENGTH));
            CubicSpine::SIG_L * (
                if q <= 0.5 {
                    q * (3.0 * q - 2.0) * gradq
                } else {
                    let factor = 1.0 - q;
                    (-factor * factor) * gradq
                }
            )
        }

        fn radius() -> f32 {
            SMOOTHING_LENGTH
        }

        fn radius_sq() -> f32 {
            SMOOTHING_LENGTH_SQ
        }
    }
}
