use nalgebra::Vector3;
use rayon::prelude::*;

use crate::{DFSPH, external_forces::ExternalForce};
use crate::utils::orthogonal_vectors;

pub struct BasicViscosityForce {
    viscosity_coeffcient: f32,
    surface_viscosity_coefficient: f32, 
}

impl BasicViscosityForce {
    pub fn new(viscosity_coeffcient: f32, surface_viscosity_coefficient: f32) -> Box<BasicViscosityForce> {
        Box::new(BasicViscosityForce {
            viscosity_coeffcient: viscosity_coeffcient,
            surface_viscosity_coefficient: surface_viscosity_coefficient,
        })
    }
}

impl ExternalForce for BasicViscosityForce {
    fn init(&mut self, _: &DFSPH) { }

    fn compute_acceleration(&self, sim: &DFSPH, accelerations: &mut Vec<Vector3<f32>>) -> f32 {
        let densities = sim.density.read().unwrap();
        let positions = sim.positions.read().unwrap();
        let velocities = sim.velocities.read().unwrap();

        let h = sim.kernel_radius();
        let h2 = sim.kernel_radius().powi(2) / 100.;

        accelerations.par_iter_mut().enumerate().for_each(|(i, v)| {
            *v += sim.neighbours_reduce_v(i, &|r, i, j| {
                let vij = velocities[i] - velocities[j];
                let xij = positions[i] - positions[j];

                let grad = sim.gradient(positions[i], positions[j]);
                let volume = sim.mass(j) / densities[j];

                r + 10. * self.viscosity_coeffcient * volume * vij.dot(&xij) / (xij.norm_squared() + h2) * grad
            });

            if self.surface_viscosity_coefficient != 0.0 {
                *v += sim.solids_reduce_v(i,  &|solid, r, vol, p| {
                    let normal = p - positions[i];
                    let norm = normal.norm();

                    if norm < 0.0001 {
                        return r;
                    }

                    let normal = normal / norm;
                    let mut result = Vector3::zeros();

                    for v in orthogonal_vectors(normal) {
                        let x = p + v * (h - norm);
                        let xij = positions[i] - x;

                        let grad = sim.gradient(positions[i], x);
                        let volume = vol * 0.25;

                        let vij = velocities[i] - solid.point_velocity(x);

                        let a = 10. * self.surface_viscosity_coefficient * volume * vij.dot(&xij) / (xij.norm_squared() + h2) * grad;

                        solid.add_force(x, -a * sim.mass(i));
                        result += a;
                    }

                    r + result
                });
            }
        });

        sim.time_step
    }
}
