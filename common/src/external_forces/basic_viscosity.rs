use nalgebra::Vector3;
use rayon::prelude::*;

use crate::{DFSPH, external_forces::ExternalForce};

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
    fn compute_acceleration(&self, sim: &DFSPH, accelerations: &mut Vec<Vector3<f32>>) {
        let densities = sim.density.read().unwrap();
        let positions = sim.positions.read().unwrap();
        let velocities = sim.velocities.read().unwrap();

        let h2 = sim.kernel_radius().powi(2) / 100.;

        accelerations.par_iter_mut().enumerate().for_each(|(i, v)| {
            *v += sim.neighbours_reduce_v(i, &|r, i, j| {
                let vij = velocities[i] - velocities[j];
                let xij = positions[i] - positions[j];

                let grad = sim.gradient(positions[i], positions[j]);
                let volume = sim.mass(j) / densities[j];

                r + 10. * self.viscosity_coeffcient * volume * vij.dot(&xij) / (xij.norm_squared() + h2) * grad
            });
        });
    }
}
