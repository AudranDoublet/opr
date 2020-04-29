use std::sync::RwLock;

use nalgebra::{Vector3, Matrix3};
use rayon::prelude::*;

use crate::Fluid;
use crate::{Simulation, external_forces::ExternalForce};

use utils::ConjugateGradientSolver;
use utils::orthogonal_vectors;

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

    fn matrix_vec_prod(&self, fluid: &Fluid, sim: &Simulation, vec: &Vec<Vector3<f32>>) -> Vec<Vector3<f32>> {
        let densities = sim.density.read().unwrap();
        let positions = sim.positions.read().unwrap();
        let dt = sim.time_step();

        let h = sim.kernel_radius();
        let h2 = sim.kernel_radius().powi(2) / 100.;

        fluid.filter_range(sim).map(|i| {
            let mut result = sim.neighbours_reduce_v(true, i, &|r, i, j| {
                let xij = positions[i] - positions[j];
                let vij = vec[fluid.correspondance(i)] - vec[fluid.correspondance(j)];

                let grad = sim.gradient(positions[i], positions[j]);
                let volume = sim.mass(j) / densities[j];

                r + 10. * self.viscosity_coeffcient * volume * vij.dot(&xij) / (xij.norm_squared() + h2) * grad
            });

            if self.surface_viscosity_coefficient != 0.0 {
                result += sim.solids_reduce_v(i,  &|solid, r, vol, p| {
                    let normal = p - positions[i];
                    let norm = normal.norm();

                    if norm < 0.0001 {
                        return r;
                    }

                    let normal = normal / norm;
                    let mut result = Vector3::zeros();

                    for v in orthogonal_vectors(normal) {
                        let x = p + v * (h / 2.);
                        let xij = positions[i] - x;

                        let grad = sim.gradient(positions[i], x);
                        let volume = vol * 0.25;

                        let vij = vec[fluid.correspondance(i)] - solid.point_velocity(x);

                        let a = 10. * self.surface_viscosity_coefficient * volume * vij.dot(&xij) / (xij.norm_squared() + h2) * grad;

                        solid.add_force(x, -a * (sim.mass(i) / densities[i]));
                        result += a;
                    }

                    r + result
                });
            }

            vec[fluid.correspondance(i)] - (dt / densities[i]) * result
        }).collect()
    }

    fn compute_preconditions(&self, fluid: &Fluid, sim: &Simulation) -> Vec<Matrix3<f32>> {
        let densities = sim.density.read().unwrap();
        let positions = sim.positions.read().unwrap();
        let dt = sim.time_step();

        let h = sim.kernel_radius();
        let h2 = sim.kernel_radius().powi(2) / 100.;

        fluid.filter(sim, positions.par_iter()).map(|(i, xi)| {
            let mut result = sim.neighbours_reduce(true, i, Matrix3::zeros(), &|r, _, j| {
                let xij = *xi - positions[j];

                let grad = sim.gradient(*xi, positions[j]);
                let volume = sim.mass(j) / densities[j];

                r + 10. * self.viscosity_coeffcient * volume / (xij.norm_squared() + h2) * (grad * xij.transpose())
            });

            if self.surface_viscosity_coefficient != 0.0 {
                result += sim.solids_reduce(i, Matrix3::zeros(), &|_, r, vol, p| {
                    let normal = p - *xi;
                    let norm = normal.norm();

                    if norm < 0.0001 {
                        return r;
                    }

                    let normal = normal / norm;
                    let mut result = Matrix3::zeros();

                    for v in orthogonal_vectors(normal) {
                        let x = p + v * (h / 2.);
                        let xij = *xi - x;

                        let grad = sim.gradient(*xi, x);
                        let volume = vol * 0.25;

                        result += 10. * self.surface_viscosity_coefficient * volume / (xij.norm_squared() + h2) * (grad * xij.transpose());
                    }

                    r + result
                });
            }

            Matrix3::identity() - (dt / densities[i]) * result
        }).collect()
    }

    fn compute_guess(&self, fluid: &Fluid, sim: &Simulation) -> Vec<Vector3<f32>> {
        let difference = self.difference.read().unwrap();

        fluid.filter(sim, sim.velocities.read().unwrap().par_iter())
            .map(|(i, v)| v + difference.get(fluid.correspondance(i)).unwrap_or(&Vector3::zeros()))
            .collect()
    }
}

impl ExternalForce for ViscosityWeiler2018Force {
    fn init(&mut self, _: &Fluid, _: &Simulation) { }

    fn compute_acceleration(&self, fluid: &Fluid, sim: &Simulation, accelerations: &mut Vec<Vector3<f32>>) -> Option<f32> {
        let mut preconditions = self.compute_preconditions(fluid, sim);
        let mut b = fluid.filter(sim, sim.velocities.read().unwrap().par_iter()).map(|(_, v)| *v).collect();
        let mut guess = self.compute_guess(fluid, sim);

        let dt = sim.time_step();

        let result = self.solver.solve(
            &|v| self.matrix_vec_prod(fluid, sim, v), &mut b, &mut guess, &mut preconditions
        );

        let velocities = sim.velocities.read().unwrap();

        *self.difference.write().unwrap() = fluid.filter(sim, velocities.par_iter())
                                                  .map(|(i, v)| result[fluid.correspondance(i)] - v).collect();

        let difference = self.difference.read().unwrap();
        fluid.filter_m(sim, accelerations.par_iter_mut()).for_each(|(i, a)| *a += (1. / dt) * difference[fluid.correspondance(i)]);
        None
    }
}
