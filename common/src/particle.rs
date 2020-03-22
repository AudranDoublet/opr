extern crate nalgebra;
extern crate serde;

use std::fs::File;
use std::io::{BufWriter, BufReader};

use flate2::Compression;
use flate2::write::ZlibEncoder;
use flate2::read::ZlibDecoder;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

const STIFFNESS: f32 = 2. / 1000.;
const STIFFNESS_NEAR: f32 = STIFFNESS * 10.;
const EPSILON: f32 = 1.;
const SIGMA: f32 = 0.2;
const BETA: f32 = 0.2;

#[derive(Copy, Clone, Serialize, Deserialize, Debug)]
pub struct ParticleNeighbour
{
    pub id: usize,
    pub dist: f32,
    pub sq_dist: f32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Particle
{
    pub position: Vector3<f32>,
    prev_position: Vector3<f32>,
    pub density: f32,
    density_near: f32,
    pub neighbours: Vec<ParticleNeighbour>,
    pressure: f32,
    pressure_near: f32,
    force: Vector3<f32>,
    pub velocity: Vector3<f32>,
}

impl Particle
{
    fn new(x: f32, y: f32, z: f32) -> Particle
    {
        Particle {
            position: Vector3::new(x, y, z),
            prev_position: Vector3::new(0., 0., 0.),
            neighbours: Vec::new(),
            density: 0.0,
            density_near: 0.0,
            pressure: 0.0,
            pressure_near: 0.0,
            force: Vector3::zeros(),
            velocity: Vector3::zeros(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Scene
{
    pub particles: Vec<Particle>,
    pub particle_radius: f32,
    sq_particle_radius: f32,
    pub rest_density: f32,
    gravity: Vector3<f32>,
    pub size: f32,
    spring_const: f32,
}

impl Scene
{
    pub fn new() -> Scene
    {
        Scene {
            particles: Vec::new(),
            particle_radius: 1.,
            sq_particle_radius: 1.,
            rest_density: 3.,
            gravity: Vector3::new(0., -0.005, 0.),
            size: 20.0,
            spring_const: 1. / 8.,
        }
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

    pub fn particle_dx(&self, id: usize) -> (f32, f32, f32)
    {
        let vec = &self.particles[id].position - &self.particles[id].prev_position;

        (vec.x, vec.y, vec.z)
    }

    pub fn add_particle(&mut self, x: f32, y: f32, z: f32)
    {
        self.particles.push(Particle::new(x, y, z))
    }

    pub fn fill_part(&mut self, fx: f32, fy: f32, fz: f32, px: f32, py: f32, pz: f32)
    {
        let epsize = self.size - 2. * EPSILON;
        let jx = (fx * epsize / self.particle_radius) as usize;
        let jy = (fy * epsize / self.particle_radius) as usize;
        let jz = (fz * epsize / self.particle_radius) as usize;
        let cx = 2 * (px * epsize / self.particle_radius) as usize + jx;
        let cy = 2 * (py * epsize / self.particle_radius) as usize + jy;
        let cz = 2 * (pz * epsize / self.particle_radius) as usize + jz;

        let radius = self.particle_radius * 0.5;

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

    fn compute_density(&mut self, id: usize, neighbours: Vec<ParticleNeighbour>)
    {
        let particle = &mut self.particles[id];
        particle.neighbours.clear();

        particle.density = 0.;
        particle.density_near = 0.;

        for mut n in neighbours {
            // replace dist by weighted dist
            n.dist = 1. - n.dist / self.particle_radius;
            n.sq_dist = n.dist * n.dist;

            particle.density += n.sq_dist;
            particle.density_near += n.sq_dist * n.dist;
            particle.neighbours.push(n);
        }
    }

    fn compute_neighbours(&self, id: usize) -> Vec<ParticleNeighbour>
    {
        let particle = &self.particles[id];
        let mut vec = Vec::new();

        for i in 0..self.particles.len() {
            let sq_dist = (&self.particles[i].position - &particle.position).norm_squared();

            if i != id && sq_dist <= self.sq_particle_radius {
                vec.push(ParticleNeighbour {
                    id: i,
                    dist: sq_dist.sqrt(),
                    sq_dist: 0.,
                });
            }
        }

        vec
    }

    fn compute_pressure(&mut self, id: usize)
    {
        let particle = &mut self.particles[id];
        particle.pressure = STIFFNESS * (particle.density - self.rest_density);
        particle.pressure_near = STIFFNESS_NEAR * particle.density_near;
    }

    fn compute_forces(&mut self, id: usize)
    {
        for neighbour in &self.particles[id].neighbours.to_owned() {
            let nid = neighbour.id;

            let par_self = &self.particles[id];
            let par_neib = &self.particles[nid];

            let dir = &par_neib.position - &par_self.position;

            // pressure
            let dm = par_self.pressure * 2. * neighbour.dist +
                (par_self.pressure_near + par_neib.pressure_near) * neighbour.sq_dist;

            let force_diff = dir.normalize() * dm;

            // viscosity
            let q = dir.norm() / self.particle_radius;
            let dir = dir.normalize();

            // FIXME why viscosity impacts directly velocity and not forces ?
            let velocity_diff = match (&par_self.velocity - &par_neib.velocity).dot(&dir)
            {
                u if u > 0. => (1. - q) * (SIGMA * u + BETA * u * u) * dir * 0.5,
                _ => Vector3::zeros(),
            };

            // apply
            self.particles[nid].force += force_diff;
            self.particles[id].force -= force_diff;
            self.particles[nid].velocity += velocity_diff;
            self.particles[id].velocity -= velocity_diff;
        }
    }

    fn apply_forces(&mut self, id: usize)
    {
        let particle = &mut self.particles[id];

        particle.prev_position = particle.position;
        particle.position += particle.velocity + particle.force;

        particle.force = self.gravity;
        particle.velocity = particle.position - particle.prev_position;

        // apply boundaries
        particle.force.x -= self.spring_const * match particle.position.x {
            v if v < EPSILON => v - EPSILON,
            v if v > self.size - EPSILON => v + EPSILON - self.size,
            _ => 0.0,
        };

        particle.force.y -= self.spring_const * match particle.position.y {
            v if v < EPSILON => v - EPSILON,
            v if v > self.size - EPSILON => v + EPSILON - self.size,
            _ => 0.0,
        };

        particle.force.z -= self.spring_const * match particle.position.z {
            v if v < EPSILON => v - EPSILON,
            v if v > self.size - EPSILON => v + EPSILON - self.size,
            _ => 0.0,
        };
    }

    pub fn tick(&mut self)
    {
        for particle in 0..self.particles.len() {
            self.apply_forces(particle);
        }

        for particle in 0..self.particles.len() {
            let neighbours = self.compute_neighbours(particle);
            self.compute_density(particle, neighbours);
        }

        for particle in 0..self.particles.len() {
            self.compute_pressure(particle);
        }

        for particle in 0..self.particles.len() {
            self.compute_forces(particle);
        }
    }

    pub fn dump(&self, path: &str) -> Result<(), std::io::Error> {
        let buffer = BufWriter::new(File::create(path)?);
        let encoder = ZlibEncoder::new(buffer, Compression::default());
        serde_json::to_writer(encoder,self)?;

        Ok(())
    }

    pub fn load(path: &str) -> Result<Scene, std::io::Error> {
        let buffer = BufReader::new(File::open(path)?);
        let decoder = ZlibDecoder::new(buffer);
        let r = serde_json::from_reader(decoder)?;

        Ok(r)
    }
}
