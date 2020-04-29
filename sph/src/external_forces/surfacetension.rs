use nalgebra::Vector3;
use rayon::prelude::*;

use utils::kernels::{Kernel, AdhesionKernel, CohesionKernel};

use crate::{Simulation, external_forces::ExternalForce};

pub struct SurfaceTensionForce {
    adhesion_kernel: AdhesionKernel,
    cohesion_kernel: CohesionKernel,
    surface_tension: f32,
    surface_adhesion: f32,
}

impl SurfaceTensionForce {
    pub fn new(kernel_radius: f32, surface_tension: f32, surface_adhesion: f32) -> Box<SurfaceTensionForce> {
        Box::new(SurfaceTensionForce {
            surface_tension: surface_tension,
            surface_adhesion: surface_adhesion,
            adhesion_kernel: AdhesionKernel::new(kernel_radius),
            cohesion_kernel: CohesionKernel::new(kernel_radius),
        })
    }
}

impl ExternalForce for SurfaceTensionForce {
    fn init(&mut self, _: &Simulation) { }

    fn compute_acceleration(&self, sim: &Simulation, accelerations: &mut Vec<Vector3<f32>>) -> Option<f32> {
        let positions = sim.positions.read().unwrap();
        let densities = sim.density.read().unwrap();
        let h = sim.kernel_radius();

        let mut normals = vec![Vector3::zeros(); positions.len()];
        normals.par_iter_mut().enumerate().for_each(|(i, v)| {
            *v = h * sim.neighbours_reduce_v(i, &|r, i, j| {
                r + sim.mass(j) / densities[j] * sim.gradient(positions[i], positions[j])
            });
        });

        accelerations.par_iter_mut().enumerate().for_each(|(i, v)| {
            // cohesion & curvature
            *v += sim.neighbours_reduce_v(i, &|r, i, j| {
                let kij = 2. * sim.rest_density / (densities[i] + densities[j]);
                let xij = positions[i] - positions[j];
                let normsq = xij.norm_squared();

                let mut accel = Vector3::zeros();

                if normsq > 1e-9 {
                    let norm = normsq.sqrt();
                    accel -= self.surface_tension * sim.mass(j) * (xij / norm) * self.cohesion_kernel.apply_on_norm(norm);
                }

                accel -= (normals[i] - normals[j]) * self.surface_tension;

                r + accel * kij
            });

            // adhesion
            *v += sim.solids_reduce_v(i, &|_, total, v, p| {
                let xij = positions[i] - p;

                let normsq = xij.norm_squared();

                total - self.surface_adhesion * (if normsq > 1e-9 {
                    let norm = normsq.sqrt();
                    v * sim.rest_density * (xij / norm) * self.adhesion_kernel.apply_on_norm(norm)
                } else {
                    Vector3::zeros()
                })
            });
        });

        None
    }
}
