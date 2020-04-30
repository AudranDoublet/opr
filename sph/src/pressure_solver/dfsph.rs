use std::sync::RwLock;
use nalgebra::Vector3;

use rayon::prelude::*;

use crate::pressure_solver::PressureSolver;
use crate::Simulation;

const EPSILON: f32 = 1e-5;

pub struct DFSPH {
    stiffness: RwLock<Vec<f32>>,
    density_prediction: RwLock<Vec<f32>>,

    correct_density_max_error: f32,
    correct_divergence_max_error: f32,
}

impl DFSPH {
    pub fn new() -> Box<DFSPH> {
        Box::new(DFSPH {
            stiffness: RwLock::new(Vec::new()),
            density_prediction: RwLock::new(Vec::new()),

            correct_density_max_error: 0.01,
            correct_divergence_max_error: 0.1,
        })
    }

    fn init_arrays(&self, len: usize) {
        let mut stiff = self.stiffness.write().unwrap();

        if stiff.len() != len {
            *stiff = vec![0.0; len];
            *self.density_prediction.write().unwrap() = vec![0.0; len];
        }
    }

    fn compute_stiffness(&self, sim: &Simulation) {
        let positions = sim.positions.read().unwrap();

        self.stiffness
            .write().unwrap()
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, v)| *v = {
                let pos = positions[i];

                let sum_a = sim.neighbours_reduce_v(false, i, &|r, _, j| r + sim.volume(j) * sim.gradient(pos, positions[j]));
                let sum_b = sim.neighbours_reduce_f(false, i, &|r, _, j| {
                    r + (sim.volume(j) * sim.gradient(pos, positions[j])).norm_squared()
                });

                // boundaries
                let sum_a = sum_a + sim.solids_reduce_v(i, &|_, total, v, p| total + v * sim.gradient(pos, p));
                let sum = sum_a.norm_squared() + sum_b;

                match sum {
                    sum if sum > EPSILON => -1.0 / sum,
                    _ => 0.0,
                }
            });
    }

    fn compute_density_variation(&self, sim: &Simulation) {
        let velocities = sim.velocities.read().unwrap();
        let positions = sim.positions.read().unwrap();

        self.density_prediction.write().unwrap().par_iter_mut().enumerate().for_each(|(i, p)| {
            let pos = positions[i];

            let density_adv = if sim.neighbours_count(i) < 20 {
                0.0
            } else {
                let mut delta = sim.neighbours_reduce_f(false, i, &|r, i, j| r + sim.volume(j) * (velocities[i] - velocities[j]).dot(&sim.gradient(pos, positions[j])));
                delta += sim.solids_reduce_f(i, &|volume, r, v, x| {
                    let vj = volume.point_velocity(x);
                    r + v * (velocities[i] - vj).dot(&sim.gradient(pos, x))
                });

                delta.max(0.0)
            };

            *p = density_adv;
        });
    }

    fn compute_density_advection(&self, sim: &Simulation) {
        let velocities = sim.velocities.read().unwrap();
        let positions = sim.positions.read().unwrap();
        let densities = sim.density.read().unwrap();

        let dt = sim.time_step();

        self.density_prediction.write().unwrap().par_iter_mut().enumerate().for_each(|(i, p)| {
            let pos = positions[i];

            let mut delta = sim.neighbours_reduce_f(false, i, &|r, i, j| r + sim.volume(j) * (velocities[i] - velocities[j]).dot(&sim.gradient(pos, positions[j])));
            delta += sim.solids_reduce_f(i, &|volume, r, v, x| {
                let vj = volume.point_velocity(x);
                r + v * (velocities[i] - vj).dot(&sim.gradient(pos, x))
            });

            *p = (densities[i] / sim.rest_density(i) + dt * delta).max(1.0);
        });
    }

    fn correct_divergence_error(&self, sim: &Simulation) {
        self.compute_density_variation(sim);

        let mut iter_count = 0;
        let mut chk = false;
        let time_step = sim.time_step();
        let step = 1. / time_step;

        while (!chk || iter_count <= 1) && iter_count < 100 {
            {
                let positions = sim.positions.read().unwrap();
                let density_adv = self.density_prediction.read().unwrap();
                let stiffness = self.stiffness.read().unwrap();

                sim.velocities.write().unwrap().par_iter_mut().enumerate().for_each(|(i, v)| {
                    if sim.fixed[i] {
                        return;
                    }

                    let ki = density_adv[i] * stiffness[i] * step;

                    let diff = sim.neighbours_reduce_v(false, i, &|r, i, j| {
                        let kj = density_adv[j] * stiffness[j] * step;
                        let sum = ki + kj * (sim.rest_density(j) / sim.rest_density(i));

                        if sum.abs() <= EPSILON {
                            return r;
                        }

                        let grad = -sim.volume(j) * sim.gradient(positions[i], positions[j]);
                        r - time_step * sum * grad
                    });

                    let boundary_diff = sim.solids_reduce_v(i, &|solid, r, v, x| {
                        let grad = -v * sim.gradient(positions[i], x);
                        let change = 1.0 * ki * grad;

                        solid.add_force(x, change * sim.mass(i));
                        r + (-time_step * change)
                    });

                    *v += diff + boundary_diff;
                });
            }

            self.compute_density_variation(sim);

            chk = true;

            for m in &sim.fluid_types {
                let density_div_avg: f32 = self.density_prediction.read()
                    .unwrap().par_iter()
                    .enumerate()
                    .filter(|(i, _)| sim.particles_fluid_type[*i] == m.idx())
                    .map(|(i, v)| v * sim.rest_density(i) - sim.rest_density(i))
                    .sum::<f32>() / sim.len() as f32;

                let eta = 1. / time_step * self.correct_divergence_max_error * 0.01 * m.rest_density();

                chk &= density_div_avg < eta;
            }

            iter_count += 1;
        }
    }

    fn correct_density_error(&self, sim: &Simulation) {
        self.compute_density_advection(sim);

        let mut iter_count = 0;
        let mut chk = false;
        let time_step = sim.time_step();
        let step = 1. / time_step.powi(2);

        while (!chk || iter_count <= 1) && iter_count < 1000 {
            {
                let positions = sim.positions.read().unwrap();
                let density_adv = self.density_prediction.read().unwrap();
                let stiffness = self.stiffness.read().unwrap();

                sim.velocities.write().unwrap().par_iter_mut().enumerate().for_each(|(i, v)| {
                    if sim.fixed[i] {
                        return;
                    }

                    let ki = (density_adv[i] - 1.) * stiffness[i] * step;

                    let diff = sim.neighbours_reduce_v(false, i, &|r, i, j| {
                        let kj = (density_adv[j] - 1.) * stiffness[j] * step;
                        let sum = ki + kj * (sim.rest_density(j) / sim.rest_density(i));

                        if sum.abs() <= EPSILON {
                            return r;
                        }

                        let grad = -sim.volume(j) * sim.gradient(positions[i], positions[j]);

                        r - time_step * sum * grad
                    });

                    let boundary_diff = match ki.abs() {
                        v if v > EPSILON => sim.solids_reduce_v(i, &|solid, r, v, x| {
                            let grad = -v * sim.gradient(positions[i], x);
                            let change = 1.0 * ki * grad;

                            solid.add_force(x, change * sim.mass(i));

                            r + (-time_step * change)
                        }),
                        _ => Vector3::zeros(),
                    };

                    *v += diff + boundary_diff;
                });
            }

            self.compute_density_advection(sim);

            chk = true;

            for m in &sim.fluid_types {
                let density_avg: f32 = self.density_prediction.read()
                    .unwrap().par_iter()
                    .enumerate()
                    .filter(|(i, _)| sim.particles_fluid_type[*i] == m.idx())
                    .map(|(i, v)| v * sim.rest_density(i) - sim.rest_density(i))
                    .sum::<f32>() / sim.len() as f32;

                let eta = self.correct_density_max_error * 0.01 * m.rest_density();

                chk &= density_avg < eta;
            }

            iter_count += 1;
        }
    }

    fn update_accelerations_with_delta_v(&self, sim: &Simulation, old: Vec<Vector3<f32>>) {
        let time_step = sim.time_step();
        let velocities = sim.velocities.read().unwrap();

        sim.accelerations.write().unwrap().par_iter_mut()
            .enumerate()
            .for_each(|(i, a)| *a += (velocities[i] - old[i]) / time_step);
    }
}

impl PressureSolver for DFSPH {
    fn time_step(&self, sim: &Simulation) {
        // init
        self.init_arrays(sim.len());
        self.compute_stiffness(sim);

        self.correct_divergence_error(sim);

        sim.adapt_cfl();
        sim.compute_non_pressure_forces();

        let velocities_copy = sim.velocities.read().unwrap().clone();
        self.correct_density_error(sim);
        self.update_accelerations_with_delta_v(sim, velocities_copy);
    }
}
