extern crate nalgebra;

use nalgebra::Vector3;

use crate::kernel::{Kernel, kernels, SMOOTHING_LENGTH};

// -- Constant Density Solver --
const CDS_SOLVER_MIN_DIVERGENCE: f32 = 0.1; // Minimum divergence error allowed for average density variation
const CDS_SOLVER_MIN_ITER: usize = 1;
const CDS_SOLVER_MAX_ITER: usize = 100;
// -- Density Solver --
const DS_SOLVER_MIN_ERROR: f32 = 0.0001; // Minimum average density deviation (from rest density) 
const DS_SOLVER_MIN_ITER: usize = 2; // Maximum number of iteration to minimize density error
const DS_SOLVER_MAX_ITER: usize = 100; // Maximum number of iteration to minimize density error


// -- CFL --
const CFL_CONSTANT: f32 = 0.4; // CFL factor for adaptative time step
const CFL_MAX_TIME_STEP: f32 = 0.0005;

const SPATIAL_DIMENSION: f32 = 3.0; // Number of spatial dimensions

const REST_DENSITY: f32 = 1.;

const EPSILON: f32 = 1.;


const _DBG_PARTICLE_IDX: usize = 100;

#[derive(Copy, Clone, Debug)]
struct ParticleNeighbour
{
    id: usize,
    dist: f32,
}

#[derive(Debug)]
struct Particle
{
    acceleration: Vector3<f32>, // non-pressure accelerations

    velocity: Vector3<f32>,
    velocity_prediction: Vector3<f32>,

    density: f32,
    density_prediction: f32,
    density_deviation: f32,

    k_dfsph: f32, // constant factor that represent particle stiffness

    // --------------------------------
    position: Vector3<f32>,
    neighbours: Vec<ParticleNeighbour>,
    pressure: f32,
    mass: f32,
    kinematic_viscosity: f32,
    // FIXME
}

impl Particle
{
    // FIXME: SMOOTHING_LENGTH and REST_DENSITY should be given as args instead of being constants
    fn new(x: f32, y: f32, z: f32) -> Particle
    {
        Particle {
            acceleration: Vector3::zeros(),
            velocity_prediction: Vector3::zeros(),
            density_deviation: 0.0,
            density_prediction: REST_DENSITY,
            position: Vector3::new(x, y, z),
            k_dfsph: 0.0,
            neighbours: Vec::new(),
            density: REST_DENSITY,
            pressure: 0.0,
            mass: 0.02,
            kinematic_viscosity: 1.787 * 10e-7, // FIXME: hardcoded for water, should be given as arg
            velocity: Vector3::zeros(),
        }
    }
}

pub struct Scene
{
    velocity_max: f32, // maximum velocity of all particles in the scene

    density_avg: f32,
    density_deviation_avg: f32, // average of the derivative of density of all particles

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
        let mut scene = Scene {
            velocity_max: 0.0,
            density_avg: 0.0,
            density_deviation_avg: 0.0,

            particles: Vec::new(),
            particle_size: 0.025,
            delta_time: CFL_MAX_TIME_STEP,
            gravity: Vector3::new(0., -9.81, 0.),
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

        let radius = self.particle_size;

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
                if sq_dist >= kernels::CubicSpine::radius_sq() {
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

    fn compute_non_pressure_acceleration(&mut self, id: usize) {
        let particle = &self.particles[id];

        let mut a_viscosity = Vector3::zeros();
        let mut a_other = self.gravity;

        for neighbour_info in &particle.neighbours {
            let neighbour = &self.particles[neighbour_info.id];
            let x_ij: Vector3<f32> = &particle.position - &neighbour.position;

            let grad_ij = kernels::CubicSpine::gradient(&x_ij);

            // cf. from eq-26: divergence-free discretization of Laplace Operator
            a_viscosity += (
                (neighbour.mass / neighbour.density)
                * (&particle.velocity - &neighbour.velocity).dot(&x_ij)
                * grad_ij
            ) / x_ij.norm_squared();
        }

        a_viscosity *= particle.kinematic_viscosity * (2. * (SPATIAL_DIMENSION + 2.));

        /*if particle.position.y < EPSILON {
            a_other.y = 0.;
            a_viscosity.y = 0.;
        }*/

        //self.particles[id].acceleration = a_viscosity + a_other;
        self.particles[id].acceleration = a_other;
    }

    fn compute_k_dfsph(&mut self, id: usize) {
        let particle = &self.particles[id];

        self.particles[id].k_dfsph = particle.density.powi(0) / {
            let mut sum_a: Vector3<f32> = Vector3::zeros();
            let mut sum_b: f32 = 0.0;

            for neighbour_info in &particle.neighbours {
                let neighbour = &self.particles[neighbour_info.id];

                let x_ij: Vector3<f32> = &particle.position - &neighbour.position;
                let grad_ij = kernels::CubicSpine::gradient(&x_ij);

                let density_variation = neighbour.mass * grad_ij;

                sum_a += density_variation;
                sum_b += density_variation.norm_squared();
            }

            // cf. from eq-79
            sum_a.norm_squared() + sum_b
        }.max(1e-6);

    }

    fn compute_velocity_prediction(&mut self, id: usize) {
        let particle = &mut self.particles[id];

        particle.velocity_prediction = particle.velocity + self.delta_time * particle.acceleration;
    }

    fn compute_density_deviation(&mut self, id: usize) {
        self.particles[id].density_deviation = {
            let particle = &self.particles[id];

            let mut density_deviation = 0.0; // D(rho)/D(t)

            for neighbour_info in &particle.neighbours {
                let neighbour = &self.particles[neighbour_info.id];

                let x_ij: Vector3<f32> = &particle.position - &neighbour.position;
                let grad_ij = kernels::CubicSpine::gradient(&x_ij);

                //println!("GRAD: {}", grad_ij);

                let dn = neighbour.mass 
                    * (particle.velocity_prediction - neighbour.velocity_prediction).dot(&grad_ij);
                density_deviation += dn;
            }

            println!("DN: {}", density_deviation);
            density_deviation
        };
    }

    fn compute_density_prediction(&mut self, id: usize) {
        let particle = &mut self.particles[id];
        particle.density_prediction = (particle.density + self.delta_time * particle.density_deviation).max(REST_DENSITY);
    }

    fn fix_velocity_prediction(&mut self, id: usize) {
        let particle = &self.particles[id];

        if id == 0 {
            //println!("-------- Fix vel pred");
            //println!("{} {}", particle.pressure, particle.density);
        }

        let vp = self.delta_time * {
            let mut pressure_grad = Vector3::zeros();
            let spi = (particle.k_dfsph * (particle.density_prediction - REST_DENSITY) / self.delta_time.powi(2)).max(0.5);
            //let spi = (particle.pressure / particle.density).max(0.5);

            for neighbour_info in &particle.neighbours {
                let neighbour = &self.particles[neighbour_info.id];
                let spj = (neighbour.k_dfsph * (neighbour.density_prediction - REST_DENSITY) / self.delta_time.powi(2)).max(0.5);
                //let spj = (neighbour.pressure / neighbour.density).max(0.5);

                if id == 0 {
                    //println!("j: {}", neighbour_info.id);
                    //println!("pressures: {}, {}", particle.pressure, neighbour.pressure);
                    //println!("(spi, spj) := ({}, {})", spi, spj);
                }

                let x_ij: Vector3<f32> = &particle.position - &neighbour.position;
                let grad_ij = kernels::CubicSpine::gradient(&x_ij);

                pressure_grad += neighbour.mass * (spi + spj) * grad_ij;
            }

            pressure_grad
        };
        if id == 0 {
            //println!("{} {}", vp, self.particles[id].velocity_prediction);
        }

        //println!("VP={}",vp);
        self.particles[id].velocity_prediction -= vp;
    }

    fn fix_velocity_prediction_ultra(&mut self, id: usize) {
        let particle = &self.particles[id];

        let vp = self.delta_time * {
            let mut pressure_grad = Vector3::zeros();
            let spi = (1.0/self.delta_time) * particle.density_deviation * particle.k_dfsph / particle.density;

            for neighbour_info in &particle.neighbours {
                let neighbour = &self.particles[neighbour_info.id];
                let spj = (1.0/self.delta_time) * neighbour.density_deviation * neighbour.k_dfsph / neighbour.density;

                let x_ij: Vector3<f32> = &particle.position - &neighbour.position;
                let grad_ij = kernels::CubicSpine::gradient(&x_ij);
                //println!("grad_ij={}", grad_ij);

                pressure_grad += neighbour.mass * (spi + spj) * grad_ij;
            }

            pressure_grad
        };

        self.particles[id].velocity_prediction -= vp;
    }

    fn update_cfl(&mut self) {
        self.delta_time = CFL_CONSTANT * (self.particle_size / self.velocity_max).min(CFL_MAX_TIME_STEP);
    }

    fn update_positions(&mut self, id: usize) {
        let v = self.particles[id].velocity_prediction;
        self.particles[id].position += v * self.delta_time;
    }

    fn update_density(&mut self, id: usize) {
        self.particles[id].density = self.particles[id].density_prediction;
    }

    fn init_density(&mut self, id: usize) {
        let mut density = 0.;

        for ninfo in &self.particles[id].neighbours {
            density += self.particles[ninfo.id].mass + kernels::CubicSpine::apply_on_norm(ninfo.dist);
        }

        self.particles[id].density = density;
    }

    fn update_velocities(&mut self, id: usize) -> f32 {
        self.particles[id].velocity = self.particles[id].velocity_prediction;
        self.particles[id].velocity.norm()
    }
    
    fn _debug_particle(&self, id: usize, header: String) {
        let particle = &self.particles[id];
        println!("{} density={} | pressure={} | k_dfspg={} | velocity={} | acceleration={}", header, particle.density, particle.pressure, particle.k_dfsph, particle.velocity, particle.acceleration);
    }

    fn correct_density_error(&mut self) {
        // correct density error cf. alg-5
        println!("> [CALL][CORRECT_DENSITY_ERROR]");
        let mut ds_iter: usize = 0;
        while (self.density_avg - REST_DENSITY > DS_SOLVER_MIN_ERROR && ds_iter < DS_SOLVER_MAX_ITER) || (ds_iter < DS_SOLVER_MIN_ITER) {
            println!(">{}: avg_density= {}", ds_iter, self.density_avg);
            //println!("//////////////////// Loop");

            for particle in 0..self.particles.len() {
                self.compute_density_deviation(particle);
                self.compute_density_prediction(particle);
            }

            self.density_avg = self.particles.iter().fold(0.0, |acc, particle| acc + particle.density_prediction)
                / (self.particles.len() as f32);


            for particle in 0..self.particles.len() {
                self.fix_velocity_prediction(particle);
            }

            ds_iter += 1;
        }
        println!(">{}: avg_density= {}", ds_iter, self.density_avg);
        //println!(">{}: (avg_density): {}", ds_iter, self.density_avg);
    }

    fn correct_divergence_error(&mut self) {
        // correct divergence error
        let mut ds_iter: usize = 0;

        while (self.density_deviation_avg > CDS_SOLVER_MIN_DIVERGENCE && ds_iter < CDS_SOLVER_MAX_ITER) || (ds_iter < CDS_SOLVER_MIN_ITER) {
            for particle in 0..self.particles.len() {
                self.compute_density_deviation(particle);
            }

            self.density_deviation_avg = self.particles.iter().fold(0.0, |acc, particle| acc + particle.density_deviation)
                / (self.particles.len() as f32);

            for particle in 0..self.particles.len() {
                self.fix_velocity_prediction_ultra(particle);
            }

            ds_iter += 1;
        }
    }

    pub fn init(&mut self)
    {
        for particle in 0..self.particles.len() {
            self.compute_neighbours(particle);
        }

        for particle in 0..self.particles.len() {
            self.init_density(particle);
            self.compute_k_dfsph(particle);
        }
    }

    pub fn tick(&mut self)
    {
        let mut min_dist = std::f32::MAX;
        let mut min_i: usize = 0;
        let mut min_j: usize = 0;
        for i in 0..self.particles.len() {
            for j in 0..self.particles.len() {
                if i == j {
                    continue;
                }
                let dist = (self.particles[i].position-self.particles[j].position).norm();
                if dist < min_dist {
                    min_dist = dist;
                    min_i = i;
                    min_j = j;
                }
            }
        }
        println!("MIN_DIST: {} {} {}", min_dist, min_i, min_j);

        println!("================================================= ITERATION ====================================");
        println!("================================================================================================");
        for particle in 0..self.particles.len() {
            self.compute_non_pressure_acceleration(particle);
        }

        self.update_cfl();

        self._debug_particle(_DBG_PARTICLE_IDX, "[ITER_BEG]".to_string());

        for particle in 0..self.particles.len() {
            self.compute_velocity_prediction(particle);
        }

        //println!("chocolllaaaaaat {}", self.particles[0].velocity_prediction);
        self.correct_density_error();
        //println!("chocolllaaaaaatine {}", self.particles[0].velocity_prediction);

        //println!("piaou {:?}", self.particles[123].velocity_prediction);

        for particle in 0..self.particles.len() {
            self.update_positions(particle);
        }

        for particle in 0..self.particles.len() {
            self.compute_neighbours(particle);
        }

        for particle in 0..self.particles.len() {
            //self.update_density(particle);
            self.init_density(particle);
            self.compute_k_dfsph(particle);
        }

        self.correct_divergence_error();

        self.velocity_max = 0.0;
        let mut idx_fdp = 0;
        for particle in 0..self.particles.len() {
            let v_particle = self.update_velocities(particle);
            if v_particle > self.velocity_max {
                self.velocity_max = v_particle;
                idx_fdp = particle;
            }
        }

        self._debug_particle(_DBG_PARTICLE_IDX, "[ITER_END]".to_string());
    }
}
