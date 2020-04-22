use rayon::prelude::*;
use nalgebra::{Matrix3, Vector3};

pub struct ConjugateGradientSolver {
    max_iteration: usize,
    tolerance: f32,
}

impl ConjugateGradientSolver {
    pub fn new(max_iteration: usize, tolerance: f32) -> ConjugateGradientSolver {
        ConjugateGradientSolver {
            max_iteration: max_iteration,
            tolerance: tolerance,
        }
    }

    fn block_jacobi(&self, preconditions: &mut Vec<Matrix3<f32>>) {
        preconditions.par_iter_mut().for_each(|v| *v = v.try_inverse().unwrap());
    }

    fn solve_precond(&self, preconditions: &Vec<Matrix3<f32>>, b: &Vec<Vector3<f32>>) -> Vec<Vector3<f32>> {
        preconditions.par_iter().zip(b.par_iter())
                     .map(|(a, b)| a * b)
                     .collect()
    }

    fn dot(&self, a: &Vec<Vector3<f32>>, b: &Vec<Vector3<f32>>) -> f32 {
        a.par_iter().zip(b.par_iter())
                    .map(|(a, b)| a.dot(b))
                    .reduce(|| 0., |a, b| a + b)
    }

    fn sub(&self, a: &Vec<Vector3<f32>>, b: &Vec<Vector3<f32>>) -> Vec<Vector3<f32>> {
        a.par_iter().zip(b.par_iter()).map(|(a, b)| a - b).collect()
    }

    fn add_factor(&self, a: &mut Vec<Vector3<f32>>, b: &Vec<Vector3<f32>>, scalar: f32) {
        a.par_iter_mut()
            .enumerate()
            .for_each(|(i, v)| *v += b[i] * scalar);
    }

    fn error(&self, residual: &Vec<Vector3<f32>>) -> f32 {
        residual.par_iter()
                .map(|v| v.norm_squared())
                .reduce(|| 0., |a, b| a + b)
    }

    pub fn solve(&self, a_mult: &(dyn Fn(&Vec<Vector3<f32>>) -> Vec<Vector3<f32>> + Sync),
                        b: &Vec<Vector3<f32>>,
                        guess: &mut Vec<Vector3<f32>>,
                        preconditions: &mut Vec<Matrix3<f32>>) -> Vec<Vector3<f32>> {

        self.block_jacobi(preconditions);

        let b_norm = self.error(b);

        if b_norm == 0.0 {
            return guess.clone();
        }

        let mut residual = self.sub(b, &a_mult(guess));
        let threshold = self.tolerance.powi(2) * b_norm;

        if self.error(&residual) < threshold {
            return guess.clone();
        }

        let mut dir = self.solve_precond(preconditions, &residual);
        let mut abs_new = self.dot(&residual, &dir);

        for _ in 0..self.max_iteration {
            let tmp = a_mult(&dir);
            let alpha = abs_new / self.dot(&dir, &tmp);

            self.add_factor(guess, &dir, alpha);
            self.add_factor(&mut residual, &tmp, -alpha);

            if self.error(&residual) < self.tolerance {
                break;
            }

            let mut tmp = self.solve_precond(preconditions, &residual);
            let old = abs_new;

            abs_new = self.dot(&residual, &tmp);
            self.add_factor(&mut tmp, &dir, abs_new / old);

            dir = tmp;
        }

        guess.clone()
    }
}
