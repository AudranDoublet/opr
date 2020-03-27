extern crate nalgebra;
extern crate serde;

use std::fs::File;
use std::io::{BufReader, BufWriter};

use flate2::Compression;
use flate2::read::ZlibDecoder;
use flate2::write::ZlibEncoder;
use nalgebra::Vector3;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::kernel::{Kernel, kernels::CubicSpine};
use crate::mesher::types::FluidSnapshot;
use crate::RigidObject;
use std::path::Path;

const EPSILON: f32 = 1e-5;

#[derive(Serialize, Deserialize, Debug)]
pub struct Particle
{
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub acceleration: Vector3<f32>,

    pub density_prediction: f32,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct DFSPH
{
    // parameters
    kernel: CubicSpine,
    pub particle_radius: f32,
    volume: f32,

    rest_density: f32,

    correct_density_max_error: f32,
    correct_divergence_max_error: f32,

    // cfl
    cfl_min_time_step: f32,
    cfl_max_time_step: f32,
    cfl_factor: f32,

    // iteration data
    time_step: f32,

    v_max: f32,
    debug_v_max_sq: f32,
    debug_v_mean_sq: f32,

    particles: Vec<Particle>,
    solids: Vec<RigidObject>,

    // Particle data
    density: Vec<f32>,
    stiffness: Vec<f32>,
    neighbours: Vec<Vec<usize>>,
}

impl Particle
{
    pub fn new(x: f32, y: f32, z: f32) -> Particle {
        Particle {
            position: Vector3::new(x, y, z),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),

            density_prediction: 0.0,
        }
    }
}


impl FluidSnapshot for DFSPH {
    fn particles(&self) -> Vec<Vector3<f32>> {
        let mut v = vec![Vector3::zeros(); self.particles.len()];
        for i in 0..self.particles.len() {
            v[i] = self.particles[i].position;
        }
        v
    }

    fn density_at(&self, position: Vector3<f32>) -> f32 {
        let mut density = 0.;
        for i in 0..self.particles.len() {
            let sq_dist = (self.position(i) - &position).norm_squared();

            if sq_dist >= 0.001 && sq_dist <= self.kernel.radius_sq() {
                density += self.volume(i) * self.kernel.apply_on_norm(sq_dist.sqrt());
            }
        }
        density
    }
}

impl DFSPH
{
    pub fn new(kernel_radius: f32, particle_radius: f32, solids: Vec<RigidObject>) -> DFSPH
    {
        DFSPH {
            kernel: CubicSpine::new(kernel_radius),
            particle_radius,
            volume: 4. * std::f32::consts::PI * particle_radius.powi(3) / 3., // FIXME how to compute it ? 5.12e-5

            rest_density: 1000.0,

            correct_density_max_error: 0.001,
            correct_divergence_max_error: 0.01,

            // time step
            cfl_min_time_step: 0.0001,
            cfl_max_time_step: 0.005,
            cfl_factor: 1.0,
            time_step: 0.0001,

            v_max: 0.0,
            debug_v_max_sq: 0.0,
            debug_v_mean_sq: 0.0,

            solids,
            particles: Vec::new(),

            neighbours: Vec::new(),
            density: Vec::new(),
            stiffness: Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        self.particles.clear();
    }

    pub fn solid_count(&self) -> usize {
        self.solids.len()
    }

    pub fn solid(&self, i: usize) -> &RigidObject {
        &self.solids[i]
    }

    pub fn debug_get_v_mean_sq  (&self) -> f32 {
        self.debug_v_mean_sq
    }

    pub fn debug_get_v_max_sq  (&self) -> f32 {
        self.debug_v_max_sq
    }

    pub fn get_v_max(&self) -> f32 {
        self.v_max
    }

    pub fn get_time_step(&self) -> f32 {
        self.time_step
    }

    pub fn len(&self) -> usize {
        self.particles.len()
    }

    pub fn particle(&self, i: usize) -> (f32, f32, f32) {
        let v = self.position(i);
        (v.x, v.y, v.z)
    }

    pub fn add_particle(&mut self, x: f32, y: f32, z: f32)
    {
        self.particles.push(Particle::new(x, y, z))
    }

    fn gradient(&self, i: usize, j: usize) -> Vector3<f32> {
        self.kernel.gradient(&(&self.particles[i].position - &self.particles[j].position))
    }

    fn gradient_solid(&self, i: usize, j: Vector3<f32>) -> Vector3<f32> {
        self.kernel.gradient(&(&self.particles[i].position - &j))
    }

    fn kernel_apply(&self, i: usize, j: usize) -> f32 {
        self.kernel.apply_on_norm((self.position(i) - self.position(j)).norm())
    }

    fn kernel_apply_solid(&self, i: usize, j: Vector3<f32>) -> f32 {
        self.kernel.apply_on_norm((self.position(i) - j).norm())
    }

    fn distance_sq(&self, i: usize, j: usize) -> f32 {
        (self.position(i) - self.position(j)).norm_squared()
    }

    fn position(&self, i: usize) -> &Vector3<f32> {
        &self.particles[i].position
    }

    pub fn particle_radius(&self) -> f32 {
        self.particle_radius
    }

    fn volume(&self, _i: usize) -> f32 {
        self.volume
    }

    fn stiffness(&self, i: usize) -> f32 {
        self.stiffness[i]
    }

    fn density(&self, i: usize) -> f32 {
        self.density[i]
    }

    fn density_adv(&self, i: usize) -> f32 {
        self.particles[i].density_prediction
    }

    pub fn velocity(&self, i: usize) -> Vector3<f32> {
        self.particles[i].velocity
    }

    pub fn acceleration(&self, i: usize) -> Vector3<f32> {
        self.particles[i].acceleration
    }

    fn neighbours_count(&self, i: usize) -> usize {
        self.neighbours[i].len()
    }

    fn neighbours_reduce<V>(&self, i: usize, value: V, f: &dyn Fn(V, usize, usize) -> V) -> V {
        let mut result = value;

        for j in 0..self.neighbours_count(i) {
            result = f(result, i, self.neighbours[i][j]);
        }

        result
    }

    fn solids_reduce<V>(&self, i: usize, value: V, f: &dyn Fn(V, f32, Vector3<f32>) -> V) -> V {
        let mut result = value;

        for s in &self.solids {
            let volume = s.particle_volume(i);

            if volume > 0.0 {
                result = f(result, volume, s.particle_boundary_x(i));
            }
        }

        result
    }

    fn neighbours_reduce_v(&self, i: usize, f: &dyn Fn(Vector3<f32>, usize, usize) -> Vector3<f32>) -> Vector3<f32> {
        self.neighbours_reduce(i, Vector3::zeros(), f)
    }

    fn neighbours_reduce_f(&self, i: usize, f: &dyn Fn(f32, usize, usize) -> f32) -> f32 {
        self.neighbours_reduce(i, 0.0, f)
    }

    fn solids_reduce_v(&self, i: usize, f: &dyn Fn(Vector3<f32>, f32, Vector3<f32>) -> Vector3<f32>) -> Vector3<f32> {
        self.solids_reduce(i, Vector3::zeros(), f)
    }

    fn solids_reduce_f(&self, i: usize, f: &dyn Fn(f32, f32, Vector3<f32>) -> f32) -> f32 {
        self.solids_reduce(i, 0.0, f)
    }

    fn compute_neighbours(&self, i: usize) -> Vec<usize> {
        let mut neighbours = Vec::new();

        for j in 0..self.len() {
            // we can ignore particles at that distance since they are discarded by the kernel
            if j == i || self.distance_sq(i, j) >= self.kernel.radius_sq() {
                continue;
            }

            neighbours.push(j);
        }

        neighbours
    }

    fn adapt_cfl(&mut self) {
        // Compute max velocity
        let mut v_max: f32 = 0.0;
        let mut debug_v_mean_sq: f64 = 0.;

        for i in 0..self.len() {
            let n = self.velocity(i).norm_squared();
            debug_v_mean_sq += n as f64;
            v_max = v_max.max(n);
        }

        self.debug_v_mean_sq = (debug_v_mean_sq / self.len() as f64) as f32;
        self.debug_v_max_sq = v_max;
        self.v_max = v_max.sqrt();

        self.time_step = ((self.cfl_factor * self.particle_radius) / self.v_max)
            .max(self.cfl_min_time_step)
            .min(self.cfl_max_time_step);
    }

    fn compute_density(&self, i: usize) -> f32 {
        let self_dens = 0.0;
        //let self_dens = self.volume(i) * self.kernel_apply(i, i); // FIXME
        let neighbour_dens = self.neighbours_reduce_f(i, &|density, i, j| {
            density + self.volume(j) * self.kernel_apply(i, j)
        });
        let solids_dens = self.solids_reduce_f(i, &|r, v, x| r + v * self.kernel_apply_solid(i, x));

        (self_dens + neighbour_dens + solids_dens) * self.rest_density
    }

    fn compute_stiffness(&self, i: usize) -> f32 {
        let sum_a = self.neighbours_reduce_v(i, &|r, i, j| r + self.volume(j) * self.gradient(i, j));
        let sum_b = self.neighbours_reduce_f(i, &|r, i, j| {
            r + (self.volume(j) * self.gradient(i, j)).norm_squared()
        });

        // boundaries
        let sum_a = sum_a + self.solids_reduce_v(i, &|total, v, p| total + v * self.gradient_solid(i, p));
        let sum = sum_a.norm_squared() + sum_b;

        match sum {
            sum if sum > EPSILON => -1.0 / sum,
            _ => 0.0,
        }
    }

    fn compute_density_variation(&mut self) {
        for i in 0..self.len() {
            let density_adv = if self.neighbours_count(i) < 20 {
                0.0
            } else {
                let mut delta = self.neighbours_reduce_f(i, &|r, i, j| r + self.volume(j) * (self.velocity(i) - self.velocity(j)).dot(&self.gradient(i, j)));
                delta += self.solids_reduce_f(i, &|r, v, x| {
                    let vj = Vector3::zeros(); //FIXME compute velocity for moving solids
                    r + v * (self.velocity(i) - vj).dot(&self.gradient_solid(i, x))
                });

                delta.max(0.0)
            };

            self.particles[i].density_prediction = density_adv;
        }
    }

    fn compute_density_advection(&mut self) {
        for i in 0..self.len() {
            let mut delta = self.neighbours_reduce_f(i, &|r, i, j| r + self.volume(j) * (self.velocity(i) - self.velocity(j)).dot(&self.gradient(i, j)));
            delta += self.solids_reduce_f(i, &|r, v, x| {
                let vj = Vector3::zeros(); //FIXME compute velocity for moving solids
                r + v * (self.velocity(i) - vj).dot(&self.gradient_solid(i, x))
            });

            self.particles[i].density_prediction = (self.density(i) / self.rest_density + self.time_step * delta).max(1.0);
        }
    }

    fn correct_density_error(&mut self) {
        self.compute_density_advection();

        let mut iter_count = 0;
        let mut chk = false;

        while (!chk || iter_count <= 1) && iter_count < 1000 {
            for i in 0..self.len() {
                let ki = (self.density_adv(i) - 1.) * self.stiffness(i) / self.time_step.powi(2);

                let diff = self.neighbours_reduce_v(i, &|r, i, j| {
                    let sum = ki + (self.density_adv(j) - 1.) * self.stiffness(j) / self.time_step.powi(2);

                    if sum.abs() <= EPSILON {
                        return r;
                    }

                    let grad = -self.volume(j) * self.gradient(i, j);

                    r - self.time_step * sum * grad
                });

                let boundary_diff = match ki.abs() {
                    v if v > EPSILON => self.solids_reduce_v(i, &|r, v, x| {
                        let grad = -v * self.gradient_solid(i, x);
                        //FIXME add force to solid
                        r + (-self.time_step * 1.0 * ki * grad)
                    }),
                    _ => Vector3::zeros(),
                };

                self.particles[i].velocity += diff + boundary_diff;
            }

            self.compute_density_advection();

            let mut density_avg = 0.0;

            for i in 0..self.len() {
                density_avg += self.density_adv(i) * self.rest_density - self.rest_density;
            }

            density_avg /= self.len() as f32;

            let eta = self.correct_density_max_error * 0.01 * self.rest_density;

            chk |= density_avg < eta;
            iter_count += 1;
        }
    }

    fn correct_divergence_error(&mut self) {
        self.compute_density_variation();

        let mut iter_count = 0;
        let mut chk = false;

        while (!chk || iter_count <= 1) && iter_count < 100 {
            for i in 0..self.len() {
                let ki = self.density_adv(i) * self.stiffness(i) / self.time_step;

                let diff = self.neighbours_reduce_v(i, &|r, i, j| {
                    let sum = ki + self.density_adv(j) * self.stiffness(j) / self.time_step;

                    if sum.abs() <= EPSILON {
                        return r;
                    }

                    let grad = -self.volume(j) * self.gradient(i, j);
                    r - self.time_step * sum * grad
                });

                let boundary_diff = self.solids_reduce_v(i, &|r, v, x| {
                    let grad = -v * self.gradient_solid(i, x);
                    //FIXME add force to solid
                    r + (-self.time_step * 1.0 * ki * grad)
                });

                self.particles[i].velocity += diff + boundary_diff;
            }

            self.compute_density_variation();

            let mut density_div_avg = 0.0;

            for i in 0..self.len() {
                density_div_avg += self.density_adv(i) * self.rest_density - self.rest_density;
            }

            density_div_avg /= self.len() as f32;

            let eta = 1. / self.time_step * self.correct_divergence_max_error * 0.01 * self.rest_density;

            chk |= density_div_avg < eta;
            iter_count += 1;
        }
    }

    fn compute_non_pressure_forces(&mut self, i: usize) {
        self.particles[i].acceleration = Vector3::zeros();
        self.particles[i].acceleration += Vector3::new(0.0, -9.81, 0.0);
    }

    fn predict_velocity(&mut self, i: usize) {
        let acc = self.acceleration(i);
        self.particles[i].velocity += acc * self.time_step;
    }

    fn update_position(&mut self, i: usize) {
        let vel = self.velocity(i);
        self.particles[i].position += self.time_step * vel;
    }

    fn init(&mut self) {
        self.neighbours = (0..self.len()).into_par_iter().map(|i| self.compute_neighbours(i)).collect();

        self.init_boundaries();

        self.density = (0..self.len()).into_par_iter().map(|i| self.compute_density(i)).collect();
        self.stiffness = (0..self.len()).into_par_iter().map(|i| self.compute_stiffness(i)).collect();
    }

    fn init_boundaries(&mut self) {
        let pr = self.particle_radius;
        let kr = self.kernel.radius();
        let dt = self.time_step;

        for solid in &mut self.solids {
            let (volume, boundary_x): (Vec<f32>, Vec<Vector3<f32>>) =
                self.particles.par_iter_mut().map(|v| solid.compute_volume_and_boundary_x(v, pr, kr, dt)).unzip();

            solid.set_volume_and_boundary_x(volume, boundary_x);
        }
    }

    pub fn tick(&mut self) {
        self.init();

        self.correct_divergence_error();

        for i in 0..self.len() {
            self.compute_non_pressure_forces(i);
        }

        self.adapt_cfl();

        for i in 0..self.len() {
            self.predict_velocity(i);
        }

        self.correct_density_error();

        for i in 0..self.len() {
            self.update_position(i);
        }
    }

    pub fn dump(&self, path: &Path) -> Result<(), std::io::Error> {
        let buffer = BufWriter::new(File::create(path)?);
        let encoder = ZlibEncoder::new(buffer, Compression::default());
        serde_json::to_writer(encoder, self)?;

        Ok(())
    }

    pub fn load(path: &str) -> Result<DFSPH, std::io::Error> {
        let buffer = BufReader::new(File::open(path)?);
        let decoder = ZlibDecoder::new(buffer);
        let r = serde_json::from_reader(decoder)?;

        Ok(r)
    }
}
