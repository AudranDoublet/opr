use std::sync::RwLock;

use nalgebra::{Vector3, Matrix3};
use rayon::prelude::*;

use crate::{DFSPH, external_forces::ExternalForce};
use crate::utils::ConjugateGradientSolver;

pub struct ViscosityWeiler2018Force {
    solver: ConjugateGradientSolver,
    viscosity_coeffcient: f32,
    surface_viscosity_coefficient: f32,

    difference: RwLock<Vec<Vector3<f32>>>,
}

impl ViscosityWeiler2018Force {
    pub fn new(viscosity_coeffcient: f32, surface_viscosity_coefficient: f32, max_iteration: usize, tolerance: f32) -> Box<ViscosityWeiler2018Force> {
        Box::new(ViscosityWeiler2018Force {
            solver: ConjugateGradientSolver::new(max_iteration, tolerance),
            difference: RwLock::new(Vec::new()),

            viscosity_coeffcient: viscosity_coeffcient,
            surface_viscosity_coefficient: surface_viscosity_coefficient,
        })
    }

    fn matrix_vec_prod(&self, sim: &DFSPH, vec: &Vec<Vector3<f32>>) -> Vec<Vector3<f32>> {
        let densities = sim.density.read().unwrap();
        let positions = sim.positions.read().unwrap();

        let h2 = sim.kernel_radius().powi(2) / 100.;

        vec.par_iter().enumerate().map(|(i, v)| {
            let result = sim.neighbours_reduce_v(i, &|r, i, j| {
                let xij = positions[i] - positions[j];
                let vij = vec[i] - vec[j];

                let grad = sim.gradient(positions[i], positions[j]);
                let volume = sim.mass(j) / densities[j];

                r + 10. * self.viscosity_coeffcient * volume * vij.dot(&xij) / (xij.norm_squared() + h2) * grad
            });

            v - (sim.time_step / densities[i]) * result
        }).collect()
    }

    fn compute_preconditions(&self, sim: &DFSPH) -> Vec<Matrix3<f32>> {
        let densities = sim.density.read().unwrap();
        let positions = sim.positions.read().unwrap();

        let h2 = sim.kernel_radius().powi(2) / 100.;

        (0..sim.len()).into_par_iter().map(|i| {
            let result = sim.neighbours_reduce(i, Matrix3::zeros(), &|r, i, j| {
                let xij = positions[i] - positions[j];

                let grad = sim.gradient(positions[i], positions[j]);
                let volume = sim.mass(j) / densities[j];

                r + 10. * self.viscosity_coeffcient * volume / (xij.norm_squared() + h2) * (grad * xij.transpose())
            });

            Matrix3::identity() - (sim.time_step / densities[i]) * result
        }).collect()
    }

    fn compute_guess(&self, sim: &DFSPH) -> Vec<Vector3<f32>> {
        let difference = self.difference.read().unwrap();

        sim.velocities.read().unwrap().par_iter()
            .enumerate()
            .map(|(i, v)| v + difference.get(i).unwrap_or(&Vector3::zeros()))
            .collect()
    }
}

impl ExternalForce for ViscosityWeiler2018Force {
    fn compute_acceleration(&self, sim: &DFSPH, accelerations: &mut Vec<Vector3<f32>>) {
        let mut preconditions = self.compute_preconditions(sim);
        let mut b = sim.velocities.read().unwrap().clone();
        let mut guess = self.compute_guess(sim);

        let result = self.solver.solve(
            &|v| self.matrix_vec_prod(sim, v), &mut b, &mut guess, &mut preconditions
        );

        let velocities = sim.velocities.read().unwrap();

        *self.difference.write().unwrap() = result.par_iter().enumerate().map(|(i, v)| v - velocities[i]).collect();

        let difference = self.difference.read().unwrap();
        accelerations.par_iter_mut().enumerate().for_each(|(i, a)| *a += (1. / sim.time_step) * difference[i]);
    }
}
