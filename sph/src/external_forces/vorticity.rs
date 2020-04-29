use std::sync::RwLock;

use nalgebra::Vector3;
use rayon::prelude::*;

use crate::Fluid;
use crate::{Simulation, external_forces::ExternalForce};

/*
 * Vorticity based on Bender 2017: Turbulent Micropolar SPH Fluids with Foam
 */
pub struct VorticityForce {
    inertia_inverse: f32,
    viscosity_omega: f32,
    vorticity_coefficient: f32,

    omegas: RwLock<Vec<Vector3<f32>>>,
    ang_velocity: RwLock<Vec<Vector3<f32>>>,
}

impl VorticityForce {
    pub fn new(vorticity_coefficient: f32, inertia_inverse: f32, viscosity_omega: f32) -> Box<VorticityForce> {
        Box::new(VorticityForce {
            omegas: RwLock::new(Vec::new()),
            ang_velocity: RwLock::new(Vec::new()),

            vorticity_coefficient: vorticity_coefficient,
            inertia_inverse: inertia_inverse,
            viscosity_omega: viscosity_omega,
        })
    }
}

impl ExternalForce for VorticityForce {
    fn init(&mut self, _: &Fluid, _: &Simulation) { }

    fn compute_acceleration(&self, fluid: &Fluid, sim: &Simulation, accelerations: &mut Vec<Vector3<f32>>) -> Option<f32> {
        let densities = sim.density.read().unwrap();
        let positions = sim.positions.read().unwrap();
        let velocities = sim.velocities.read().unwrap();
        let mut omegas = self.omegas.write().unwrap();

        let dt = sim.time_step();

        let mut ang_velocity = self.ang_velocity.write().unwrap();
        *ang_velocity = vec![Vector3::zeros(); sim.len()];

        fluid.filter_mt(sim, accelerations.par_iter_mut().zip(ang_velocity.par_iter_mut()))
          .for_each(|(i, (a, ang_vel))| {
            *a += sim.neighbours_reduce_v(true, i, &|r, _, j| {
                let omegaij = omegas.get(i).unwrap_or(&Vector3::zeros()) - omegas.get(j).unwrap_or(&Vector3::zeros());
                let grad = sim.gradient(positions[i], positions[j]);

                r + (1./densities[i]) * self.vorticity_coefficient * sim.mass(j) * omegaij.cross(&grad)
            });

            *ang_vel += sim.neighbours_reduce_v(true, i, &|r, _, j| {
                let vij = velocities[i] - velocities[j];
                let omegaij = omegas.get(i).unwrap_or(&Vector3::zeros()) - omegas.get(j).unwrap_or(&Vector3::zeros());
                let grad = sim.gradient(positions[i], positions[j]);
                let w = sim.kernel_apply(positions[i], positions[j]);

                let volumej = sim.mass(j) / densities[j];

                r - (1./dt) * self.inertia_inverse * self.viscosity_omega * volumej * omegaij * w
                  + (1./densities[i]) * self.inertia_inverse * self.vorticity_coefficient * sim.mass(j) * vij.cross(&grad)
            });

            *a += sim.solids_reduce_v(i, &|solid, total, v, p| {
                let omegaij = *omegas.get(i).unwrap_or(&Vector3::zeros());
                let grad = sim.gradient(positions[i], p);

                let a = self.vorticity_coefficient * 1. / densities[i] * sim.rest_density(i) * v * omegaij.cross(&grad);
                solid.add_force(p, -sim.mass(i) * a);

                total + a
            });

            *ang_vel += sim.solids_reduce_v(i, &|solid, total, v, p| {
                let vij = velocities[i] - solid.point_velocity(p);
                let grad = sim.gradient(positions[i], p);

                total + self.vorticity_coefficient * (1. / densities[i]) * self.inertia_inverse * (sim.rest_density(i) * v * vij.cross(&grad))
            });

            *ang_vel -= 2. * self.inertia_inverse * self.vorticity_coefficient * omegas.get(i).unwrap_or(&Vector3::zeros());
        });

        *omegas = ang_velocity.par_iter()
            .enumerate()
            .map(|(i, v)| omegas.get(i).unwrap_or(&Vector3::zeros()) + v * dt)
            .collect();

        None
    }
}
