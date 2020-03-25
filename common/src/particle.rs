use rayon::prelude::*;

use nalgebra::Vector3;
use crate::kernel::{kernels::CubicSpine, Kernel};
use crate::RigidObject;

const EPSILON : f32 = 1e-5;

#[derive(Debug)]
pub struct Particle
{
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    pub acceleration: Vector3<f32>,

    pub stiffness: f32,
    pub density: f32,
    pub density_prediction: f32,
    pub neighbours: Vec<usize>,
}

pub struct DFSPH
{
    // parameters
    kernel: CubicSpine,
    pub particle_radius: f32,

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

    particles: Vec<Particle>,
    solids: Vec<RigidObject>,
}

impl Particle
{
    pub fn new(x: f32, y: f32, z: f32) -> Particle {
        Particle {
            position: Vector3::new(x, y, z),
            velocity: Vector3::zeros(),
            acceleration: Vector3::zeros(),

            stiffness: 0.0, // initialized by init()
            density: 0.0, // initialized by init()
            neighbours: Vec::new(), // initialized by init()

            density_prediction: 0.0,
        }
    }
}

impl DFSPH
{
    pub fn new(kernel_radius: f32, particle_radius: f32, solids: Vec<RigidObject>) -> DFSPH
    {
        DFSPH {
            kernel: CubicSpine::new(kernel_radius),
            particle_radius: particle_radius,

            rest_density: 1000.0, //FIXME

            correct_density_max_error: 0.001,
            correct_divergence_max_error: 0.01,

            // time step
            cfl_min_time_step: 0.0001,
            cfl_max_time_step: 0.005,
            cfl_factor: 1.0,
            time_step: 0.0001,

            v_max: 0.0,

            solids: solids,
            particles: Vec::new(),
        }
    }

    pub fn clear(&mut self) { self.particles.clear(); }

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

    fn mass(&self, _i: usize) -> f32 {
        self.particle_radius.powi(3)
    }

    fn volume(&self, _i: usize) -> f32 {
        //self.particle_radius.powi(3)
        5.12e-5
    }

    fn stiffness(&self, i: usize) -> f32 {
        self.particles[i].stiffness
    }

    fn density(&self, i: usize) -> f32 {
        self.particles[i].density
    }

    fn density_adv(&self, i: usize) -> f32 {
        self.particles[i].density_prediction
    }

    fn velocity(&self, i: usize) -> Vector3<f32> {
        self.particles[i].velocity
    }

    fn acceleration(&self, i: usize) -> Vector3<f32> {
        self.particles[i].acceleration
    }

    fn neighbours_count(&self, i: usize) -> usize {
        self.particles[i].neighbours.len()
    }

    fn neighbours_reduce<V>(&self, i: usize, value: V, f: &dyn Fn(V, usize, usize) -> V) -> V {
        let mut result = value;

        for j in 0..self.neighbours_count(i) {
            result = f(result, i, self.particles[i].neighbours[j]);
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

    fn compute_neighbours(&mut self, i: usize) {
        let neighbours = {
            let mut neighbours = Vec::new();

            for j in 0..self.len() {
                // we can ignore particles at that distance since they are discarded by the kernel
                if j == i || self.distance_sq(i, j) >= self.kernel.radius_sq() {
                    continue;
                }

                neighbours.push(j);
            }

            neighbours
        };

        self.particles[i].neighbours = neighbours;
    }

    fn adapt_cfl(&mut self) {
        // Compute max velocity
        let mut v_max: f32 = 0.0;

        for i in 0..self.len() {
            v_max = v_max.max(self.velocity(i).norm_squared());
        }

        self.v_max = v_max.sqrt();

        self.time_step = ((self.cfl_factor * self.particle_radius) / self.v_max)
            .max(self.cfl_min_time_step)
            .min(self.cfl_max_time_step);
    }

    fn compute_density(&mut self, i: usize) {
        let self_dens = self.volume(i) * self.kernel_apply(i, i);
        let neighbour_dens = self.neighbours_reduce_f(i, &|density, i, j| {
            density + self.volume(j) * self.kernel_apply(i, j)
        });
        let solids_dens = self.solids_reduce_f(i, &|r, v, x| r + v * self.kernel_apply_solid(i, x));

        self.particles[i].density = (self_dens + neighbour_dens + solids_dens) * self.rest_density; // mass
    }

    fn compute_stiffness(&mut self, i: usize) {
        let sum_a = self.neighbours_reduce_v(i, &|r, i, j| r + self.volume(j) * self.gradient(i, j));
        let sum_b = self.neighbours_reduce_f(i, &|r, i, j| {
            r + (self.volume(j) * self.gradient(i, j)).norm_squared()
        });

        // boundaries
        let sum_a = sum_a + self.solids_reduce_v(i, &|total, v, p| total + v * self.gradient_solid(i, p));
        let sum = sum_a.norm_squared() + sum_b;

        self.particles[i].stiffness = match sum {
            sum if sum > EPSILON => -1.0 / sum,
            _ => 0.0,
        };
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

                    //println!("sum: {}" ,sum );
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

                    r + self.time_step * self.volume(j) * sum * self.gradient(i, j)
                });

                let boundary_diff = self.solids_reduce_v(i, &|r, v, x| {
                    let grad = -v * self.gradient_solid(i, x);
                    //FIXME add force to solid
                    r + self.time_step * 1.0 * ki * grad
                });

                self.particles[i].velocity -= diff + boundary_diff;
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

    fn minmax(&self, name: &str, f: &dyn Fn(usize) -> f32) {
        let mut min = std::f32::INFINITY;
        let mut max = std::f32::NEG_INFINITY;

        for i in 0..self.len() {
            min = min.min(f(i));
            max = max.max(f(i));
        }

        println!("{} min {} max {}", name, min, max);
    }

    fn init(&mut self) {
        for i in 0..self.len() {
            self.compute_neighbours(i);
        }

        self.init_boundaries();

        for i in 0..self.len() {
            self.compute_density(i);
            self.compute_stiffness(i);
        }
    }

    fn init_boundaries(&mut self) {
        let pr = self.particle_radius;
        let kr = self.kernel.radius();
        let dt = self.time_step;

        for solid in &mut self.solids {
            let (volume, boundary_x) : (Vec<f32>, Vec<Vector3<f32>>) =
               self.particles.par_iter_mut().map(|v| solid.compute_volume_and_boundary_x(v, pr, kr, dt)).unzip();

            solid.set_volume_and_boundary_x(volume, boundary_x);
        }
    }

    pub fn tick(&mut self) {
        self.init();

        //self.correct_divergence_error();

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
}
