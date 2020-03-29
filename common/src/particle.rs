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
use crate::{RigidObject, HashGrid};
use std::path::Path;

use std::sync::RwLock;

const EPSILON: f32 = 1e-5;

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

    solids: Vec<RigidObject>,

    // Particle data
    density: RwLock<Vec<f32>>,
    stiffness: RwLock<Vec<f32>>,
    neighbours: Vec<Vec<usize>>,
    density_prediction: RwLock<Vec<f32>>,
    pub velocities: RwLock<Vec<Vector3<f32>>>,
    pub positions: RwLock<Vec<Vector3<f32>>>,
    pub accelerations: RwLock<Vec<Vector3<f32>>>,

    len: usize,

    neighbours_struct: HashGrid,
}

/*
use std::time::Instant;

macro_rules! timeit {
    ($name:expr, $code:expr) => ({
        let now = Instant::now();
        let result = $code;

        println!("{} : {}ms", $name, now.elapsed().as_micros() as f32 / 1000.);

        result
    })
}
*/

impl FluidSnapshot for DFSPH {
    fn particles(&self) -> Vec<Vector3<f32>> {
        self.positions.read().unwrap().clone()
    }

    fn density_at(&self, position: Vector3<f32>) -> f32 {
        let positions = self.positions.read().unwrap();

        let neighbours = self.neighbours_struct.find_neighbours(self.len(), &*positions, position);
        let mut density = 0.;

        for i in neighbours {
            density += self.volume(i) * self.kernel_apply(positions[i], position);
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

            correct_density_max_error: 0.01,
            correct_divergence_max_error: 0.1,

            // time step
            cfl_min_time_step: 0.0001,
            cfl_max_time_step: 0.005,
            cfl_factor: 1.0,
            time_step: 0.0001,

            v_max: 0.0,
            debug_v_max_sq: 0.0,
            debug_v_mean_sq: 0.0,

            solids,

            neighbours: Vec::new(),
            density: RwLock::new(Vec::new()),
            stiffness: RwLock::new(Vec::new()),

            len: 0,

            neighbours_struct: HashGrid::new(kernel_radius),
            density_prediction: RwLock::new(Vec::new()),
            accelerations: RwLock::new(Vec::new()),
            velocities: RwLock::new(Vec::new()),
            positions: RwLock::new(Vec::new()),
        }
    }

    pub fn clear(&mut self) {
        self.accelerations.write().unwrap().clear();
        self.velocities.write().unwrap().clear();
        self.positions.write().unwrap().clear();
        self.density_prediction.write().unwrap().clear();
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
        self.len
    }

    pub fn add_particle(&mut self, x: f32, y: f32, z: f32)
    {
        self.density_prediction.write().unwrap().push(0.0);
        self.positions.write().unwrap().push(Vector3::new(x, y, z));
        self.velocities.write().unwrap().push(Vector3::zeros());
        self.accelerations.write().unwrap().push(Vector3::zeros());
        self.stiffness.write().unwrap().push(0.0);
        self.density.write().unwrap().push(0.0);
        self.len += 1;
    }

    fn gradient(&self, i: Vector3<f32>, j: Vector3<f32>) -> Vector3<f32> {
        self.kernel.gradient(&(i - j))
    }

    fn kernel_apply(&self, i: Vector3<f32>, j: Vector3<f32>) -> f32 {
        self.kernel.apply_on_norm((i - j).norm())
    }

    pub fn particle_radius(&self) -> f32 {
        self.particle_radius
    }

    fn volume(&self, _i: usize) -> f32 {
        self.volume
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

    fn adapt_cfl(&mut self) -> f32 {
        // Compute max velocity
        let mut v_max: f32 = 0.0;
        let mut debug_v_mean_sq: f64 = 0.;

        let velocities = self.velocities.read().unwrap();

        for i in 0..self.len() {
            let n = velocities[i].norm_squared();
            debug_v_mean_sq += n as f64;
            v_max = v_max.max(n);
        }

        self.debug_v_mean_sq = (debug_v_mean_sq / self.len() as f64) as f32;
        self.debug_v_max_sq = v_max;
        self.v_max = v_max.sqrt();

        self.time_step = ((self.cfl_factor * self.particle_radius) / self.v_max)
            .max(self.cfl_min_time_step)
            .min(self.cfl_max_time_step);

        self.time_step
    }

    fn compute_density(&self, i: usize, positions: &Vec<Vector3<f32>>) -> f32 {
        let pos = positions[i];

        let self_dens = 0.0;

        let neighbour_dens = self.neighbours_reduce_f(i, &|density, _, j| {
            density + self.volume(j) * self.kernel_apply(pos, positions[j])
        });

        let solids_dens = self.solids_reduce_f(i, &|r, v, x| r + v * self.kernel_apply(pos, x));

        (self_dens + neighbour_dens + solids_dens) * self.rest_density
    }

    fn compute_stiffness(&self, i: usize, positions: &Vec<Vector3<f32>>) -> f32 {
        let pos = positions[i];

        let sum_a = self.neighbours_reduce_v(i, &|r, _, j| r + self.volume(j) * self.gradient(pos, positions[j]));
        let sum_b = self.neighbours_reduce_f(i, &|r, _, j| {
            r + (self.volume(j) * self.gradient(pos, positions[j])).norm_squared()
        });

        // boundaries
        let sum_a = sum_a + self.solids_reduce_v(i, &|total, v, p| total + v * self.gradient(pos, p));
        let sum = sum_a.norm_squared() + sum_b;

        match sum {
            sum if sum > EPSILON => -1.0 / sum,
            _ => 0.0,
        }
    }

    fn compute_density_variation(&mut self) {
        let velocities = self.velocities.read().unwrap();
        let positions = self.positions.read().unwrap();

        self.density_prediction.write().unwrap().par_iter_mut().enumerate().for_each(|(i, p)| {
            let pos = positions[i];

            let density_adv = if self.neighbours_count(i) < 20 {
                0.0
            } else {
                let mut delta = self.neighbours_reduce_f(i, &|r, i, j| r + self.volume(j) * (velocities[i] - velocities[j]).dot(&self.gradient(pos, positions[j])));
                delta += self.solids_reduce_f(i, &|r, v, x| {
                    let vj = Vector3::zeros(); //FIXME compute velocity for moving solids
                    r + v * (velocities[i] - vj).dot(&self.gradient(pos, x))
                });

                delta.max(0.0)
            };

            *p = density_adv;
        });
    }

    fn compute_density_advection(&mut self) {
        let velocities = self.velocities.read().unwrap();
        let positions = self.positions.read().unwrap();
        let densities = self.density.read().unwrap();

        self.density_prediction.write().unwrap().par_iter_mut().enumerate().for_each(|(i, p)| {
            let pos = positions[i];

            let mut delta = self.neighbours_reduce_f(i, &|r, i, j| r + self.volume(j) * (velocities[i] - velocities[j]).dot(&self.gradient(pos, positions[j])));
            delta += self.solids_reduce_f(i, &|r, v, x| {
                let vj = Vector3::zeros(); //FIXME compute velocity for moving solids
                r + v * (velocities[i] - vj).dot(&self.gradient(pos, x))
            });

            *p = (densities[i] / self.rest_density + self.time_step * delta).max(1.0);
        });
    }

    fn correct_density_error(&mut self) {
        self.compute_density_advection();

        let mut iter_count = 0;
        let mut chk = false;
        let step = 1. / self.time_step.powi(2);

        while (!chk || iter_count <= 1) && iter_count < 1000 {
            {
                let positions = self.positions.read().unwrap();
                let density_adv = self.density_prediction.read().unwrap();
                let stiffness = self.stiffness.read().unwrap();

                self.velocities.write().unwrap().par_iter_mut().enumerate().for_each(|(i, v)| {
                    let ki = (density_adv[i] - 1.) * stiffness[i] * step;

                    let diff = self.neighbours_reduce_v(i, &|r, i, j| {
                        let sum = ki + (density_adv[j] - 1.) * stiffness[j] * step;

                        if sum.abs() <= EPSILON {
                            return r;
                        }

                        let grad = -self.volume(j) * self.gradient(positions[i], positions[j]);

                        r - self.time_step * sum * grad
                    });

                    let boundary_diff = match ki.abs() {
                        v if v > EPSILON => self.solids_reduce_v(i, &|r, v, x| {
                            let grad = -v * self.gradient(positions[i], x);
                            //FIXME add force to solid
                            r + (-self.time_step * 1.0 * ki * grad)
                        }),
                        _ => Vector3::zeros(),
                    };

                    *v += diff + boundary_diff;
                });
            }

            self.compute_density_advection();

            let density_avg: f32 = self.density_prediction.read().unwrap().par_iter()
                                    .map(|v| v * self.rest_density - self.rest_density)
                                    .sum::<f32>() / self.len() as f32;

            let eta = self.correct_density_max_error * 0.01 * self.rest_density;

            chk |= density_avg < eta;
            iter_count += 1;
        }
    }

    fn correct_divergence_error(&mut self) {
        self.compute_density_variation();

        let mut iter_count = 0;
        let mut chk = false;
        let step = 1. / self.time_step;

        while (!chk || iter_count <= 1) && iter_count < 100 {
            {
                let positions = self.positions.read().unwrap();
                let density_adv = self.density_prediction.read().unwrap();
                let stiffness = self.stiffness.read().unwrap();

                self.velocities.write().unwrap().par_iter_mut().enumerate().for_each(|(i, v)| {
                    let ki = density_adv[i] * stiffness[i] * step;

                    let diff = self.neighbours_reduce_v(i, &|r, i, j| {
                        let sum = ki + density_adv[j] * stiffness[j] * step;

                        if sum.abs() <= EPSILON {
                            return r;
                        }

                        let grad = -self.volume(j) * self.gradient(positions[i], positions[j]);
                        r - self.time_step * sum * grad
                    });

                    let boundary_diff = self.solids_reduce_v(i, &|r, v, x| {
                        let grad = -v * self.gradient(positions[i], x);
                        //FIXME add force to solid
                        r + (-self.time_step * 1.0 * ki * grad)
                    });

                    *v += diff + boundary_diff;
                });
            }

            self.compute_density_variation();

            let density_div_avg: f32 = self.density_prediction.read().unwrap().par_iter()
                                    .map(|v| v * self.rest_density - self.rest_density)
                                    .sum::<f32>() / self.len() as f32;

            let eta = 1. / self.time_step * self.correct_divergence_max_error * 0.01 * self.rest_density;

            chk |= density_div_avg < eta;
            iter_count += 1;
        }
    }

    fn compute_non_pressure_forces(&self, _i: usize, acceleration: &mut Vector3<f32>) {
        *acceleration = Vector3::new(0.0, -9.81, 0.0);
    }

    fn init(&mut self) {
        self.neighbours = self.neighbours_struct.find_all_neighbours(&self.positions.read().unwrap());

        self.init_boundaries();

        let positions = &*self.positions.read().unwrap();

        self.density.write().unwrap().par_iter_mut().enumerate().for_each(|(i, v)| *v = self.compute_density(i, positions));
         self.stiffness.write().unwrap().par_iter_mut().enumerate().for_each(|(i, v)| *v = self.compute_stiffness(i, positions));
    }

    fn init_boundaries(&mut self) {
        let pr = self.particle_radius;
        let kr = self.kernel.radius();
        let dt = self.time_step;

        for solid in &mut self.solids {
            let (volume, boundary_x): (Vec<f32>, Vec<Vector3<f32>>) =
                self.velocities.write().unwrap().par_iter_mut()
                        .zip(self.positions.write().unwrap().par_iter_mut())
                        .map(|(v, p)| solid.compute_volume_and_boundary_x(p, v, pr, kr, dt)).unzip();

            solid.set_volume_and_boundary_x(volume, boundary_x);
        }
    }

    pub fn sync(&mut self) {
        self.neighbours_struct = HashGrid::new(self.kernel.radius());
        self.neighbours_struct.insert(&self.positions.read().unwrap());
    }

    pub fn tick(&mut self) {
        self.init();

        self.correct_divergence_error();

        self.accelerations.write().unwrap().par_iter_mut().enumerate().for_each(|(i, a)| self.compute_non_pressure_forces(i, a));

        let dt = self.adapt_cfl();

        {
            let accelerations = self.accelerations.read().unwrap();
            self.velocities.write().unwrap().par_iter_mut().enumerate().for_each(|(i, v)| *v += accelerations[i] * dt);
        }

        self.correct_density_error();
        self.neighbours_struct.update_particles(self.time_step, &mut self.positions.write().unwrap(), &self.velocities.read().unwrap());
    }

    pub fn dump(&self, path: &Path) -> Result<(), std::io::Error> {
        let buffer = BufWriter::new(File::create(path)?);
        let encoder = ZlibEncoder::new(buffer, Compression::default());
        serde_json::to_writer(encoder, self)?;

        Ok(())
    }

    pub fn load(path: &Path) -> Result<DFSPH, std::io::Error> {
        let buffer = BufReader::new(File::open(path)?);
        let decoder = ZlibDecoder::new(buffer);
        let mut r : DFSPH = serde_json::from_reader(decoder)?;

        r.sync();

        Ok(r)
    }
}
