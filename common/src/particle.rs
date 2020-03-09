extern crate nalgebra;

use std::net::Shutdown::Read;

use nalgebra::Vector3;

use crate::kernel::{Kernel, kernels, SMOOTHING_LENGTH};

use self::nalgebra::Vector;

const STIFFNESS: f32 = 2. / 10.;
const REST_DENSITY: f32 = 10.;

const EPSILON: f32 = 1.;

#[derive(Copy, Clone)]
struct ParticleNeighbour
{
    id: usize,
    dist: f32,
}

struct Particle
{
    position: Vector3<f32>,
    density: f32,
    neighbours: Vec<ParticleNeighbour>,
    pressure: f32,
    mass: f32,
    kinematic_viscosity: f32,
    // FIXME
    forces: Vector3<f32>,
    velocity: Vector3<f32>,
}

impl Particle
{
    // FIXME: SMOOTHING_LENGTH and REST_DENSITY should be given as args instead of being constants
    fn new(x: f32, y: f32, z: f32) -> Particle
    {
        Particle {
            position: Vector3::new(x, y, z),
            neighbours: Vec::new(),
            density: 0.0,
            pressure: 0.0,
            mass: SMOOTHING_LENGTH.powi(3) * REST_DENSITY,
            kinematic_viscosity: 1.787 * 10e-7, // FIXME: hardcoded for water, should be given as arg
            forces: Vector3::zeros(),
            velocity: Vector3::zeros(),
        }
    }
}

pub struct Scene
{
    particles: Vec<Particle>,
    pub particle_size: f32,
    delta_time: f32,
    gravity: Vector3<f32>,
    pub size: f32,
    spring_const: f32,
}

impl Scene
{
    pub fn new() -> Scene
    {
        let scene = Scene {
            particles: Vec::new(),
            particle_size: 1.0,
            delta_time: 1.0,
            gravity: Vector3::new(0., -0.005, 0.),
            size: 20.0,
            spring_const: 1. / 16.,
        };

        scene
    }

    pub fn len(&self) -> usize
    {
        self.particles.len()
    }

    pub fn clear(&mut self)
    {
        self.particles.clear();
    }

    pub fn particle(&self, id: usize) -> (f32, f32, f32)
    {
        let vec = &self.particles[id].position;

        (vec.x, vec.y, vec.z)
    }

    pub fn add_particle(&mut self, x: f32, y: f32, z: f32)
    {
        self.particles.push(Particle::new(x, y, z))
    }

    pub fn fill_part(&mut self, fx: f32, fy: f32, fz: f32, px: f32, py: f32, pz: f32)
    {
        let epsize = self.size - 2. * EPSILON;
        let jx = (fx * epsize / self.particle_size) as usize;
        let jy = (fy * epsize / self.particle_size) as usize;
        let jz = (fz * epsize / self.particle_size) as usize;
        let cx = 2 * (px * epsize / self.particle_size) as usize + jx;
        let cy = 2 * (py * epsize / self.particle_size) as usize + jy;
        let cz = 2 * (pz * epsize / self.particle_size) as usize + jz;

        let radius = self.particle_size * 0.5;

        for x in jx..cx {
            for y in jy..cy {
                for z in jz..cz {
                    self.add_particle(
                        EPSILON + radius * x as f32,
                        EPSILON + radius * y as f32,
                        EPSILON + radius * z as f32,
                    );
                }
            }
        }
    }

    pub fn fill(&mut self, px: f32, py: f32, pz: f32)
    {
        self.fill_part(0.0, 0.0, 0.0, px, py, pz)
    }

    fn compute_density(&mut self, id: usize)
    {
         self.particles[id].density = {
            let particle = &self.particles[id];
            let mut density: f32 = 0.0;

            for n in &particle.neighbours {
                density += self.particles[n.id].mass * kernels::CubicSpine::apply_on_norm(n.dist);
            }

            density
        };
    }

    fn compute_neighbours(&mut self, id: usize)
    {
        let neighbours = {
            let particle = &self.particles[id];
            let mut neighbours = Vec::new();

            for j in 0..self.particles.len() {
                if j == id {
                    continue;
                }

                let sq_dist = (&self.particles[j].position - &particle.position).norm_squared();
                if sq_dist > kernels::CubicSpine::radius_sq() {
                    // we can ignore particles at that distance since they are discarded by the kernel
                    continue;
                }

                neighbours.push(ParticleNeighbour {
                    id: j,
                    dist: sq_dist.sqrt(),
                });
            }

            neighbours
        };

        self.particles[id].neighbours = neighbours;
    }

    fn compute_pressure(&mut self, id: usize) {
        let particle = &mut self.particles[id];
        particle.pressure = STIFFNESS * (particle.density / REST_DENSITY - 1.0).max(0.0);
    }

    fn compute_forces(&mut self, id: usize) {
        let particle = &self.particles[id];
        let pdi = particle.pressure / particle.density.powi(2); // idk why I called this variable pdi, probably because it sounds scientific (but it has no meaning)

        let mut f_pressure = Vector3::zeros();
        let mut f_viscosity = Vector3::zeros();
        let mut f_other = particle.mass * &self.gravity;

        for neighbour_info in &particle.neighbours {
            let neighbour = &self.particles[neighbour_info.id];
            let x_ij: Vector3<f32> = &particle.position - &neighbour.position;

            let grad_ij = kernels::CubicSpine::gradient(&x_ij);


            // cf. from eq-23: discretization of Laplace Operator
            f_viscosity -= (neighbour.mass / neighbour.density)
                * (&particle.velocity - &neighbour.velocity)
                * (2.0 * (grad_ij.norm() / neighbour_info.dist));

            // cf. from eq-19: discrete Lagrangian density estimate derivation
            let pdj = neighbour.pressure / neighbour.density.powi(2);
            f_pressure += neighbour.mass * (pdi + pdj) * grad_ij;
        }

        f_viscosity *= particle.mass * particle.kinematic_viscosity;
        f_pressure *= -1.0;

        // apply boundaries
        f_other.x -= self.spring_const * match particle.position.x {
            v if v < EPSILON => v - EPSILON,
            v if v > self.size - EPSILON => v + EPSILON - self.size,
            _ => 0.0,
        };

        f_other.y -= self.spring_const * match particle.position.y {
            v if v < EPSILON => v - EPSILON,
            v if v > self.size - EPSILON => v + EPSILON - self.size,
            _ => 0.0,
        };

        f_other.z -= self.spring_const * match particle.position.z {
            v if v < EPSILON => v - EPSILON,
            v if v > self.size - EPSILON => v + EPSILON - self.size,
            _ => 0.0,
        };

        self.particles[id].forces = f_pressure + f_viscosity + f_other;;
    }

    pub fn tick(&mut self)
    {
        for particle in 0..self.particles.len() {
            self.compute_neighbours(particle);
        }

        for particle in 0..self.particles.len() {
            self.compute_density(particle);
            self.compute_pressure(particle);
        }

        for particle in 0..self.particles.len() {
            self.compute_forces(particle);
        }

        let mut v_max: f32 = 0.0;
        for particle in &mut self.particles {
            particle.velocity += (self.delta_time / particle.mass) * &particle.forces;
            particle.position += self.delta_time * &particle.velocity;

            v_max = v_max.max(particle.velocity.norm());
        }
        self.delta_time = 1.;
        //self.delta_time = (0.04 * self.particle_size / v_max).min(1.0);
        println!("{} {}", self.delta_time, v_max);
    }
}
