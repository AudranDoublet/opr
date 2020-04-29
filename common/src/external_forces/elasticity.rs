/*
 * Reference: An Implicit SPH Formulation for Incompressible Linearly Elastic Solids
 */

use std::sync::RwLock;

use nalgebra::{Vector3, Vector6, Matrix3, UnitQuaternion};
use rayon::prelude::*;

use crate::{Simulation, external_forces::ExternalForce};
use crate::utils::ConjugateGradientSolver;

pub struct ElasticityForce {
    solver: ConjugateGradientSolver,

    youngs_modulus: f32,
    poisson_ratio: f32,
    alpha: f32,

    // solid constants
    rest_volumes: Vec<f32>,
    neighbours_0: Vec<Vec<usize>>,
    positions_0: Vec<Vector3<f32>>,
    l_matrices: Vec<Matrix3<f32>>,

    // solid state
    rotations: RwLock<Vec<Matrix3<f32>>>,
}

fn sym_mat_mul(mat: Vector6<f32>, v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(
        mat[0]*v[0] + mat[3]*v[1] + mat[4]*v[2],
        mat[3]*v[0] + mat[1]*v[1] + mat[5]*v[2],
        mat[4]*v[0] + mat[5]*v[1] + mat[2]*v[2],
    )
}

impl ElasticityForce {
    pub fn new(youngs_modulus: f32, poisson_ratio: f32, alpha: f32, max_iteration: usize, tolerance: f32) -> Box<ElasticityForce> {
        Box::new(ElasticityForce {
            solver: ConjugateGradientSolver::new(max_iteration, tolerance),

            youngs_modulus: youngs_modulus,
            poisson_ratio: poisson_ratio,
            alpha: alpha,

            rest_volumes: Vec::new(),
            neighbours_0: Vec::new(),
            positions_0: Vec::new(),
            l_matrices: Vec::new(),
            rotations: RwLock::new(Vec::new()),
        })
    }

    fn matrix_vec_prod(&self, sim: &Simulation, vec: &Vec<Vector3<f32>>) -> Vec<Vector3<f32>> {
        let dt = sim.time_step();
        let rotations = self.rotations.read().unwrap();

        let mu = self.youngs_modulus / (2.0 * (1.0 + self.poisson_ratio));
        let lambda = self.youngs_modulus * self.poisson_ratio
                        / ((1. + self.poisson_ratio) * (1. - 2. * self.poisson_ratio));

        let stress: Vec<Vector6<f32>> = (0..sim.len()).into_par_iter()
                  .map(|i| {
                    let mut f = Matrix3::zeros();

                    // equation 18
                    for &j in &self.neighbours_0[i] {
                        let vji = vec[j] - vec[i];
                        let grad = (rotations[i] * self.l_matrices[i]) * sim.gradient(self.positions_0[i], self.positions_0[j]);

                        f += self.rest_volumes[j] * vji * grad.transpose();
                    }
                    f *= dt;

                    // cauchy strain
                    let strain = Vector6::new(
                        f[(0,0)],
                        f[(1,1)],
                        f[(2,2)],
                        0.5 * (f[(0, 1)] + f[(1, 0)]),
                        0.5 * (f[(0, 2)] + f[(2, 0)]),
                        0.5 * (f[(1, 2)] + f[(2, 1)]),
                    );

                    let ltrace = lambda * (strain[0] + strain[1] + strain[2]);
                    let mut stress = strain * 2. * mu;

                    stress[0] += ltrace;
                    stress[1] += ltrace;
                    stress[2] += ltrace;

                    stress
                  }).collect();

        (0..sim.len()).into_par_iter()
            .map(|i| {
                let mut force = Vector3::zeros();

                for &j in &self.neighbours_0[i] {
                    let grad = sim.gradient(self.positions_0[i], self.positions_0[j]);

                    let corrected_i = (rotations[i] * self.l_matrices[i]) * grad;
                    let corrected_j = -(rotations[j] * self.l_matrices[j]) * grad;

                    let pwi = sym_mat_mul(stress[i], corrected_i);
                    let pwj = sym_mat_mul(stress[j], corrected_j);
                    force += self.rest_volumes[i] * self.rest_volumes[j] * (pwi - pwj);
                }

                vec[i] - dt * force / sim.mass(i)
            })
            .collect()
    }

    fn compute_guess(&self, sim: &Simulation, accelerations: &Vec<Vector3<f32>>) -> Vec<Vector3<f32>> {
        let dt = sim.time_step();

        sim.velocities.read().unwrap().par_iter()
            .enumerate()
            .map(|(i, v)| v + dt * accelerations[i])
            .collect()
    }

    // equation 1
    fn compute_l(&self, sim: &Simulation) -> Vec<Matrix3<f32>> {
        (0..sim.len())
            .into_par_iter()
            .map(|i| {
                let mut result = Matrix3::zeros();

                for &j in &self.neighbours_0[i] {
                    let xji0 = self.positions_0[j] - self.positions_0[i];
                    let grad = sim.gradient(self.positions_0[j], self.positions_0[i]);

                    result -= self.rest_volumes[j] * grad * xji0.transpose()
                }

                match result.try_inverse() {
                    Some(v) => v,
                    None    => Matrix3::zeros(),
                }
            })
            .collect()
    }

    fn compute_rotation(&self, sim: &Simulation) {
        let positions = sim.positions.read().unwrap();
        let mut rotations = self.rotations.write().unwrap();

        rotations.par_iter_mut()
                 .enumerate()
                 .for_each(|(i, rotation)| {
                    let mut result = Matrix3::zeros();

                    for &j in &self.neighbours_0[i] {
                        let xji = positions[j] - positions[i];
                        let grad = self.l_matrices[i] * sim.gradient(self.positions_0[i], self.positions_0[j]);

                        result += self.rest_volumes[j] * xji * grad.transpose();
                    }

                    // normalize matrix
                    *rotation = Matrix3::from(UnitQuaternion::from_matrix(&result));
                    // compute rotation * l_matrix here ?
                 });
    }

    fn compute_particle_density(&self, positions: &Vec<Vector3<f32>>, sim: &Simulation, i: usize) -> f32 {
        let xi = positions[i];
        let mut result = sim.mass(i) * sim.kernel_apply(xi, xi);

        result += sim.neighbours_reduce_f(i, &|r, _, j| {
            r + sim.mass(j) * sim.kernel_apply(xi, positions[j])
        });

        result
    }

    fn compute_rhs(&self, sim: &Simulation, accelerations: &mut Vec<Vector3<f32>>) -> Vec<Vector3<f32>> {
        let rotations = self.rotations.read().unwrap();
        let positions = sim.positions.read().unwrap();
        let velocities = sim.velocities.read().unwrap();

        let dt = sim.time_step();

        let mu = self.youngs_modulus / (2.0 * (1.0 + self.poisson_ratio));
        let lambda = self.youngs_modulus * self.poisson_ratio
                        / ((1. + self.poisson_ratio) * (1. - 2. * self.poisson_ratio));

        let (f, stress): (Vec<Matrix3<f32>>, Vec<Vector6<f32>>) = (0..sim.len()).into_par_iter()
                  .map(|i| {
                    let mut f = Matrix3::zeros();

                    // equation 18
                    for &j in &self.neighbours_0[i] {
                        let xji = positions[j] - positions[i];
                        let grad = (rotations[i] * self.l_matrices[i]) * sim.gradient(self.positions_0[i], self.positions_0[j]);

                        f += self.rest_volumes[j] * xji * grad.transpose();
                    }

                    // cauchy strain
                    let strain = Vector6::new(
                        f[(0,0)] - 1.0,
                        f[(1,1)] - 1.0,
                        f[(2,2)] - 1.0,
                        0.5 * (f[(0, 1)] + f[(1, 0)]),
                        0.5 * (f[(0, 2)] + f[(2, 0)]),
                        0.5 * (f[(1, 2)] + f[(2, 1)]),
                    );

                    let ltrace = lambda * (strain[0] + strain[1] + strain[2]);
                    let mut stress = strain * 2. * mu;

                    stress[0] += ltrace;
                    stress[1] += ltrace;
                    stress[2] += ltrace;

                    (f, stress)
                  }).unzip();

        accelerations.par_iter_mut()
            .enumerate()
            .map(|(i, acc)| {
                let mut force = Vector3::zeros();

                for &j in &self.neighbours_0[i] {
                    let grad = sim.gradient(self.positions_0[i], self.positions_0[j]);

                    let corrected_i = (rotations[i] * self.l_matrices[i]) * grad;
                    let corrected_j = -(rotations[j] * self.l_matrices[j]) * grad;

                    let pwi = sym_mat_mul(stress[i], corrected_i);
                    let pwj = sym_mat_mul(stress[j], corrected_j);
                    force += self.rest_volumes[i] * self.rest_volumes[j] * (pwi - pwj);
                }

                if self.alpha != 0.0 {
                    let mut fi_hg = Vector3::zeros();

                    for &j in &self.neighbours_0[i] {
                        let xij = positions[j] - positions[i];
                        let dist = xij.norm();

                        if dist < 1e-6 {
                            continue;
                        }

                        let xij0 = self.positions_0[j] - self.positions_0[i];
                        let dist0 = xij0.norm_squared();

                        let w = sim.kernel_apply(self.positions_0[j], self.positions_0[i]);

                        let xij_i = f[i] * rotations[i] * xij0;
                        let xij_j = -f[j] * rotations[j] * xij0;

                        let delta_i = (xij_i - xij).dot(&xij) / dist;
                        let delta_j = -(xij_j + xij).dot(&xij) / dist;

                        fi_hg -= self.rest_volumes[j] * w / dist0 * (delta_i + delta_j) * (xij / dist);
                    }

                    fi_hg *= self.alpha * self.youngs_modulus * self.rest_volumes[i];
                    *acc += fi_hg / sim.mass(i);
                }

                velocities[i] + dt * (*acc + force / sim.mass(i))
            })
            .collect()
    }
}

impl ExternalForce for ElasticityForce {
    fn init(&mut self, sim: &Simulation) {
        let positions = sim.positions.read().unwrap();

        // compute initial fluid neighbours
        self.neighbours_0 = (0..sim.len()).into_par_iter()
                                          .map(|i| sim.neighbours(i))
                                          .collect();

        self.positions_0 = positions.clone();

        self.rest_volumes = (0..sim.len()).into_par_iter()
                                          .map(|i| sim.mass(i) / self.compute_particle_density(&positions, sim, i))
                                          .collect();

        self.rotations = RwLock::new(vec![Matrix3::identity(); sim.len()]);
        self.l_matrices = self.compute_l(sim);
    }

    fn compute_acceleration(&self, sim: &Simulation, accelerations: &mut Vec<Vector3<f32>>) -> Option<f32> {
        self.compute_rotation(sim);

        let mut preconditions = Vec::new();
        let mut b = self.compute_rhs(sim, accelerations);
        let mut guess = self.compute_guess(sim, accelerations);

        let dt = sim.time_step();

        let result = self.solver.solve(
            &|v| self.matrix_vec_prod(sim, v), &mut b, &mut guess, &mut preconditions
        );

        let velocities = sim.velocities.read().unwrap();
        accelerations
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, a)| *a += (result[i] - velocities[i]) / dt);

        Some(sim.compute_cfl(&result).1)
    }
}
