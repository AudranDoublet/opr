use std::borrow::Borrow;
use std::ops::{Div, Range};
use std::sync::RwLock;

use itertools::Itertools;
use nalgebra::{RealField, Vector3};
use rand::*;
use rand::distributions::Uniform;
use rayon::prelude::*;
use sph_common::DFSPH;
use sph_common::mesher::types::VertexWorld;

use crate::diffuse_particle::{DiffuseParticle, DiffuseParticleType};
use crate::params::BubblerHyperParameters;

pub struct Bubbler {
    /// Bubbler Hyper Parameters
    hp: BubblerHyperParameters,

    /// Time elapsed between the last and the current tick
    dt: f32,
    mass_spray: f32,

    /// Store the likelihood of particles to trap air
    likelihood_ta: RwLock<Vec<f32>>,
    /// Store the likelihood of particles to be at the crest of a wave
    likelihood_wc: RwLock<Vec<f32>>,
    /// Store the likelihood of particles to generate diffuse material due to its kinematic energy
    likelihood_k: RwLock<Vec<f32>>,

    grad_densities: RwLock<Vec<Vector3<f32>>>,

    particles: Vec<DiffuseParticle>,
}

fn clamp_normalized(value: f32, range: &Range<f32>) -> f32 {
    let (min, max) = (range.start, range.end);
    (value.min(max) - value.min(min)) / (max - min)
}

fn weighting(r_norm: f32, h: f32) -> f32 {
    if r_norm <= h {
        1.0 - r_norm / h
    } else {
        0.0
    }
}

fn orthogonal_vector(v_normalized: &Vector3<f32>) -> (Vector3<f32>, Vector3<f32>) {
    assert!((v_normalized.norm() - 1.0).abs() < 0.001);

    let mut up = Vector3::y();
    if v_normalized.dot(&up) >= 0.999 {
        up = Vector3::x();
    }

    let e1 = v_normalized.cross(&up);
    let e2 = v_normalized.cross(&e1);

    (e1, e2)
}

fn make_cylinder_samplers() -> (Uniform<f32>, Uniform<f32>, Uniform<f32>) {
    (Uniform::new_inclusive(0.0, 1.0), Uniform::new_inclusive(0.0, 1.0), Uniform::new_inclusive(0.0, 1.0))
}

impl Bubbler {
    pub fn new(parameters: BubblerHyperParameters, simulation: &DFSPH) -> Self {
        // FIXME: won't work if the simulation is empty, or if the simulation contains different type of particles
        let mass_spray = parameters.mass_ratio_spray * simulation.mass(0);
        // FIXME-END

        Bubbler {
            hp: parameters,
            dt: 0.0,
            mass_spray,
            likelihood_ta: RwLock::new(vec![0.; simulation.len()]),
            likelihood_wc: RwLock::new(vec![0.; simulation.len()]),
            likelihood_k: RwLock::new(vec![0.; simulation.len()]),
            grad_densities: RwLock::new(vec![Vector3::zeros(); simulation.len()]),
            particles: vec![],
        }
    }

    pub fn get_particles(&self) -> &Vec<DiffuseParticle> {
        &self.particles
    }

    fn update_containers_capacities(&mut self, simulation: &DFSPH) {
        let new_size = simulation.len();

        self.likelihood_ta.write().unwrap().resize(new_size, 0.);
        self.likelihood_wc.write().unwrap().resize(new_size, 0.);
        self.likelihood_k.write().unwrap().resize(new_size, 0.);
        self.grad_densities.write().unwrap().resize(new_size, Vector3::zeros());
    }

    fn compute_likelihood_ta(&mut self, simulation: &DFSPH) {
        let velocities = simulation.velocities.read().unwrap();
        let positions = simulation.positions.read().unwrap();

        // FIXME: we must ignore particles that aren't near from the surface of the fluid
        // FIXME-END;
        self.likelihood_ta.write().unwrap().par_iter_mut().enumerate().for_each(|(i, potential)| {
            let v_diff = simulation.neighbours_reduce_f(i, &|v, i, j| {
                let x_ij: Vector3<f32> = &positions[i] - &positions[j];
                let v_ij: Vector3<f32> = &velocities[i] - &velocities[j];

                let w_ij = weighting(x_ij.norm(), simulation.kernel_radius());

                v + if w_ij == 0.0 { 0.0 } else { v_ij.norm() * (1.0 - v_ij.normalize().dot(&x_ij.normalize())) * w_ij }
            });

            *potential = clamp_normalized(v_diff, &self.hp.tau_ta);

            assert!(*potential >= 0. && *potential <= 1.0)
        });
    }

    fn compute_likelihood_wc(&mut self, simulation: &DFSPH) {
        let velocities = simulation.velocities.read().unwrap();
        let positions = simulation.positions.read().unwrap();
        let grad_densities = self.grad_densities.read().unwrap();

        // FIXME: we must ignore particles that aren't near from the surface of the fluid
        // FIXME-END;
        self.likelihood_wc.write().unwrap().par_iter_mut().enumerate().for_each(|(i, potential)| {
            // FIXME: is the normal to the surface of the fluid just the opposite of the density gradient?
            let n_i: Vector3<f32> = (-&grad_densities[i]).normalize();
            // FIXME-END;

            *potential =
                // We check if the velocity follows enough the normal of the surface normal, if they aren't aiming enough at the same direction
                // it's likely we're not at crest but just at an edge of the fluid (e.g. cube of water), so we can discard the particle
                if velocities[i].normalize().dot(&n_i) >= self.hp.threshold_wc_normal_direction {
                    let curvature_i = simulation.neighbours_reduce_f(i, &|acc, i, j| {
                        let x_ji: Vector3<f32> = &positions[j] - &positions[i];
                        let n_j: Vector3<f32> = (-&grad_densities[j]).normalize();

                        let curvature_ij = if x_ji.normalize().dot(&n_i) >= 0.0 {
                            0.0
                        } else { (1.0 - n_i.dot(&n_j)) * weighting(x_ji.norm(), simulation.kernel_radius()) };

                        acc + curvature_ij
                    });

                    clamp_normalized(curvature_i, &self.hp.tau_wc)
                } else {
                    0.
                };

            assert!(*potential >= 0. && *potential <= 1.0)
        });
    }

    fn compute_likelihood_k(&mut self, simulation: &DFSPH) {
        let velocities = simulation.velocities.read().unwrap();

        // FIXME: we must ignore particles that aren't near from the surface of the fluid
        // FIXME-END;
        self.likelihood_k.write().unwrap().par_iter_mut().enumerate().for_each(|(i, potential)| {
            *potential = clamp_normalized(0.5 * simulation.mass(i) * velocities[i].norm().powi(2), &self.hp.tau_k);
            assert!(*potential >= 0. && *potential <= 1.0)
        });
    }

    fn compute_grad_densities(&mut self, simulation: &DFSPH) {
        let positions = simulation.positions.read().unwrap();

        self.grad_densities.write().unwrap().par_iter_mut().enumerate().for_each(|(i, grad_density)| {
            *grad_density = simulation.neighbours_reduce_v(i, &|v, i, j| {
                v + simulation.mass(j) * simulation.gradient(positions[i], positions[j])
            });
        });
    }

    fn generate_diffuse_particles(&mut self, simulation: &DFSPH) {
        let likelihood_ta = self.likelihood_ta.read().unwrap();
        let likelihood_wc = self.likelihood_wc.read().unwrap();
        let likelihood_k = self.likelihood_k.read().unwrap();
        let positions = simulation.positions.read().unwrap();
        let velocities = simulation.velocities.read().unwrap();

        let particle_radius = simulation.particle_radius;

        self.particles.par_extend((0..simulation.len()).into_par_iter()
            .map(|i| {
                let mut rng = rand::thread_rng();
                let nb_diffuse = (likelihood_k[i] * (self.hp.k_ta * likelihood_ta[i] + self.hp.k_wc * likelihood_wc[i]) * self.dt);

                // if likelihood_k[i] > 0.5 || likelihood_ta[i] > 0.5 || likelihood_wc[i] > 0.5 {
                //     println!("{} => I_k = {}, I_ta = {}, I_wc = {} // nb_diffuse = {}", i, likelihood_k[i], likelihood_ta[i], likelihood_wc[i], nb_diffuse);
                // }

                let nb_diffuse = nb_diffuse as usize;

                let (dist_r, dist_theta, dist_h) = make_cylinder_samplers();
                let mut res = vec![];
                for _ in 0..nb_diffuse {
                    let r = particle_radius * rng.sample(dist_r).sqrt();
                    let theta = rng.sample(dist_theta) * 2.0 * f32::pi();
                    // FIXME: we use the delta time of the Bubbler, but shouldn't it rather be the delta time of the DFSPH?
                    let h = rng.sample(dist_h) * (self.dt * &velocities[i]).norm();
                    // FIXME-END;
                    let v_normalized = &velocities[i].normalize();
                    let (e1, e2) = orthogonal_vector(&v_normalized);

                    let dir = r * theta.cos() * e1 + r * theta.sin() * e2;

                    let x_d = &positions[i] + dir + h * v_normalized;
                    let v_d = &velocities[i] + dir;
                    let lifetime = self.infer_diffuse_particle_lifetime(likelihood_ta[i], likelihood_wc[i], likelihood_k[i]);

                    let neighbours = simulation.find_neighbours(&x_d);
                    res.push(DiffuseParticle { position: x_d, velocity: v_d, lifetime: lifetime, kind: self.infer_diffuse_particle_type(&neighbours) });
                }

                res
            })
            .fold(|| vec![], |mut a, b| {
                a.extend(b);
                a
            })
            .reduce(|| vec![], |mut a, b| {
                a.extend(b);
                a
            })
        );
    }

    // FIXME: not sure about this part, the paper doesn't give much information about how to compute the lifetime of a diffuse particle
    fn infer_diffuse_particle_lifetime(&self, potential_ta: f32, potential_wc: f32, potential_k: f32) -> f32 {
        let (min, max) = (self.hp.lifetime.start, self.hp.lifetime.end);
        min + (potential_ta + potential_wc + potential_k).div(3.0) * (max - min)
    }
    // FIXME-END;

    fn infer_diffuse_particle_type(&self, p_neighbours: &Vec<usize>) -> DiffuseParticleType {
        match p_neighbours.len() {
            n if self.hp.interval_neighbours_bubble.contains(&n) => DiffuseParticleType::Bubble,
            n if self.hp.interval_neighbours_spray.contains(&n) => DiffuseParticleType::Spray,
            n if self.hp.interval_neighbours_foam.contains(&n) => DiffuseParticleType::Foam,
            n => panic!(format!("the number of neighbours ({}) does not fit into any DiffuseParticleType, the ranges are:\n\
                - Bubble => {:?}\n\
                - Spray  => {:?}\n\
                - Foam   => {:?}\n", n, self.hp.interval_neighbours_bubble, self.hp.interval_neighbours_spray, self.hp.interval_neighbours_foam))
        }
    }

    fn update_diffuse_particles(&mut self, simulation: &DFSPH) {
        let positions = simulation.positions.read().unwrap();
        let velocities = simulation.velocities.read().unwrap();
        let gravity = simulation.gravity();

        let compute_avg_local_fluid_velocity = |x: &VertexWorld, neighbours: Vec<usize>| {
            let mut w_sum = 0.;
            let mut weighted_velocities = Vector3::zeros();
            for j in neighbours {
                let w = simulation.kernel_apply(*x, positions[j]);
                weighted_velocities += &velocities[j] * w;
                w_sum += w;
            }
            weighted_velocities / w_sum
        };

        let dt = self.dt;
        self.particles.par_iter_mut().for_each(|p| p.lifetime -= dt);

        self.particles = (*self.particles).into_par_iter()
            .filter(|p| !p.is_dissolved())
            .map(|p| {
                let neighbours = simulation.find_neighbours(&p.position);
                let p_type = self.infer_diffuse_particle_type(&neighbours);

                let new_velocity = match p_type {
                    // FIXME: the paper is mentioning F_ext, but which forces should we add? Perhaps we should consider the fluid acceleration as well?
                    DiffuseParticleType::Spray => &p.velocity + self.dt * gravity,
                    // FIXME-END;
                    DiffuseParticleType::Foam => compute_avg_local_fluid_velocity(&p.position, neighbours),
                    DiffuseParticleType::Bubble => &p.velocity +
                        self.dt * (-self.hp.k_b * gravity + self.hp.k_d * (&compute_avg_local_fluid_velocity(&p.position, neighbours) - &p.velocity) / self.dt),
                    _ => panic!("this won't happened"),
                };

                DiffuseParticle {
                    position: &p.position + self.dt * new_velocity,
                    velocity: new_velocity,
                    lifetime: p.lifetime,
                    kind: p_type,
                }
            })
            .fold(|| vec![], |mut a, b| {
                a.push(b);
                a
            })
            .reduce(|| vec![], |mut a, b| {
                a.extend(b);
                a
            });
    }

    pub fn tick(&mut self, simulation: &DFSPH) -> bool {
        self.dt += simulation.get_time_step();

        if self.dt < self.hp.dt_min {
            return false;
        }

        println!("curr_nb_particles           : {}", self.particles.len());
        self.update_diffuse_particles(simulation);
        let mut prev_len = self.particles.len();
        println!("nb_particles_after_filter   : {}", self.particles.len());

        self.compute_grad_densities(simulation);
        self.compute_likelihood_ta(simulation);
        self.compute_likelihood_wc(simulation);
        self.compute_likelihood_k(simulation);

        // println!("likelihood_k: {:?}", self.likelihood_k.read().unwrap());
        // println!("likelihood_ta: {:?}", self.likelihood_ta.read().unwrap());
        // println!("likelihood_wc: {:?}", self.likelihood_wc.read().unwrap());

        self.generate_diffuse_particles(simulation);
        println!("nb_newly_generated_particles: {}", self.particles.len() - prev_len);
        println!();


        self.dt = 0.0;

        true
    }
}
