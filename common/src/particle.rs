use nalgebra::Vector3;

use crate::kernel::{Kernel, kernels::CubicSpine};

const EPSILON: f32 = 1e-5;

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
    size: f32,
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
    pub fn new(size: f32) -> DFSPH
    {
        DFSPH {
            kernel: CubicSpine::new(2.), // FIXME ?
            particle_radius: 0.5, // FIXME !!
            size, // pas FIXME :D

            rest_density: 1000.0, //FIXME

            correct_density_max_error: 0.01,
            correct_divergence_max_error: 0.01,

            // time step
            cfl_min_time_step: 0.0001,
            cfl_max_time_step: 0.005,
            cfl_factor: 1.0,
            time_step: 0.0001,

            v_max: 0.0,

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

    pub fn fill_part(&mut self, fx: f32, fy: f32, fz: f32, px: f32, py: f32, pz: f32)
    {
        let epsilon = 1.0;

        let epsize = self.size - 2. * epsilon;
        let jx = (fx * epsize / self.particle_radius) as usize;
        let jy = (fy * epsize / self.particle_radius) as usize;
        let jz = (fz * epsize / self.particle_radius) as usize;
        let cx = 2 * (px * epsize / self.particle_radius) as usize + jx;
        let cy = 2 * (py * epsize / self.particle_radius) as usize + jy;
        let cz = 2 * (pz * epsize / self.particle_radius) as usize + jz;

        let radius = self.particle_radius * 1.;

        for x in jx..cx {
            for y in jy..cy {
                for z in jz..cz {
                    self.add_particle(
                        epsilon + radius * x as f32,
                        epsilon + radius * y as f32,
                        epsilon + radius * z as f32,
                    );
                }
            }
        }
    }

    pub fn fill(&mut self, px: f32, py: f32, pz: f32)
    {
        self.fill_part(0.0, 0.0, 0.0, px, py, pz)
    }

    fn gradient(&self, i: usize, j: usize) -> Vector3<f32> {
        self.kernel.gradient(&(&self.particles[i].position - &self.particles[j].position))
    }

    fn kernel_apply(&self, i: usize, j: usize) -> f32 {
        self.kernel.apply_on_norm((self.position(i) - self.position(j)).norm())
    }

    fn distance_sq(&self, i: usize, j: usize) -> f32 {
        (self.position(i) - self.position(j)).norm_squared()
    }

    fn position(&self, i: usize) -> &Vector3<f32> {
        &self.particles[i].position
    }

    fn mass(&self, _i: usize) -> f32 {
        self.particle_radius.powi(3)
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
            result = f(result, i, j);
        }

        result
    }

    fn neighbours_reduce_v(&self, i: usize, f: &dyn Fn(Vector3<f32>, usize, usize) -> Vector3<f32>) -> Vector3<f32> {
        self.neighbours_reduce(i, Vector3::zeros(), f)
    }

    fn neighbours_reduce_f(&self, i: usize, f: &dyn Fn(f32, usize, usize) -> f32) -> f32 {
        self.neighbours_reduce(i, 0.0, f)
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
        self.particles[i].density = self.neighbours_reduce_f(i, &|density, i, j| density + self.mass(j) * self.kernel_apply(i, j));
    }

    fn compute_stiffness(&mut self, i: usize) {
        let sum_a = self.neighbours_reduce_v(i, &|r, i, j| r + self.mass(j) * self.gradient(i, j));
        let sum_b = self.neighbours_reduce_f(i, &|r, i, j| r + (self.mass(j) * self.gradient(i, j)).norm_squared());

        let sum = sum_a.norm_squared() + sum_b;

        self.particles[i].stiffness = match sum {
            sum if sum > EPSILON => 1.0 / sum,
            _ => 0.0,
        };
    }

    fn compute_density_variation(&mut self) {
        for i in 0..self.len() {
            let density_adv = if self.neighbours_count(i) < 20 {
                0.0
            } else {
                let delta = self.neighbours_reduce_f(i, &|r, i, j| r + self.mass(j) * (self.velocity(i) - self.velocity(j)).dot(&self.gradient(i, j)));
                //println!("delta: {}", delta);
                delta.max(0.0)
            };

            self.particles[i].density_prediction = density_adv;
        }
    }

    fn compute_density_advection(&mut self) {
        for i in 0..self.len() {
            let delta = self.neighbours_reduce_f(i, &|r, i, j| r + self.mass(j) * (self.velocity(i) - self.velocity(j)).dot(&self.gradient(i, j)));
            // FIXME: divide by density0 ?: 
            self.particles[i].density_prediction = self.density(i) / self.rest_density + self.time_step * delta;
            //println!("{:?}", self.particles[i].density_prediction);
        }
    }

    fn correct_density_error(&mut self) {
        self.compute_density_advection();

        let mut iter_count = 0;
        let mut chk = false;

        while (!chk || iter_count <= 1) && iter_count < 1000 {
            // FIXME pk -1 ? density0 ? sami pas content
            for i in 0..self.len() {
                let ki = (self.density_adv(i) - 0.) * self.stiffness(i) / self.time_step.powi(2);

                let diff = self.neighbours_reduce_v(i, &|r, i, j| {
                    let sum = ki + (self.density_adv(j) - 0.) * self.stiffness(j) / self.time_step.powi(2);

                    //println!("sum: {}" ,sum );
                    if sum <= EPSILON {
                        return r;
                    }

                    r + self.time_step * self.mass(j) * sum * self.gradient(i, j)
                });

                self.particles[i].velocity -= diff;
            }

            self.compute_density_advection();

            let mut density_avg = 0.0;

            for i in 0..self.len() {
                //FIXME multiply by density0 ?
                density_avg += self.density_adv(i);
            }

            density_avg /= self.len() as f32;
            println!("density_avg: {}", density_avg);

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

                    if sum <= EPSILON {
                        return r;
                    }

                    r + self.time_step * self.mass(j) * sum * self.gradient(i, j)
                });

                self.particles[i].velocity -= diff;
            }

            self.compute_density_variation();

            let mut density_div_avg = 0.0;

            for i in 0..self.len() {
                //FIXME multiply by density0 ?
                density_div_avg += self.density_adv(i);
            }

            density_div_avg /= self.len() as f32;

            println!("density_div_avg: {}", density_div_avg);
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
        for i in 0..self.len() {
            self.compute_neighbours(i);
        }

        for i in 0..self.len() {
            self.compute_density(i);
            self.compute_stiffness(i);
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
}
