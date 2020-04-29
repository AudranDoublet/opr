extern crate nalgebra;
extern crate serde;

use std::sync::RwLock;

use nalgebra::Vector3;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::{Animation, Camera, Emitter};
use crate::RigidObject;
use utils::kernels::{Kernel, CubicSpine};
use mesher::types::{FluidSnapshot, FluidSnapshotProvider};

use crate::SimulationFluidSnapshot;

use crate::Fluid;
use crate::pressure_solver::*;

use search::HashGrid;

fn default_pressure_solver() -> RwLock<Box<dyn PressureSolver + Send + Sync>> {
    RwLock::new(DFSPH::new())
}

#[derive(Serialize, Deserialize)]
pub struct Simulation
{
    // parameters
    kernel: CubicSpine,
    pub particle_radius: f32,
    volume: f32,

    pub rest_density: f32,

    // cfl
    cfl_min_time_step: f32,
    cfl_max_time_step: f32,
    cfl_factor: f32,

    // iteration data
    pub time_step: RwLock<f32>,

    solids: Vec<RigidObject>,

    // Particle data
    pub density: RwLock<Vec<f32>>,
    neighbours: Vec<Vec<usize>>,
    pub velocities: RwLock<Vec<Vector3<f32>>>,
    pub positions: RwLock<Vec<Vector3<f32>>>,
    pub accelerations: RwLock<Vec<Vector3<f32>>>,

    pub particles_fluid_type: Vec<usize>,

    pub camera: Camera,

    fluid_types: Vec<Fluid>,

    #[serde(skip_serializing, skip_deserializing)]
    debug_solid_collisions: Vec<Vector3<f32>>,

    #[serde(skip_serializing, skip_deserializing)]
    #[serde(default = "default_pressure_solver")]
    pressure_solver: RwLock<Box<dyn PressureSolver + Send + Sync>>,

    #[serde(skip_serializing, skip_deserializing)]
    pub emitters: Vec<Emitter>,
    #[serde(skip_serializing, skip_deserializing)]
    pub emitters_animations: Vec<Animation>,

    #[serde(skip_serializing, skip_deserializing)]
    camera_animation: Animation,

    len: usize,

    #[serde(skip_serializing, skip_deserializing)]
    neighbours_struct: HashGrid,
}

impl FluidSnapshotProvider for Simulation {
    fn radius(&self) -> f32 {
        self.kernel.radius()
    }

    fn snapshot(&self, kernel_radius: f32, anisotropic_radius: Option<f32>) -> Box<dyn FluidSnapshot> {
        let particles = self.positions.read().unwrap().clone();
        let densities = self.density.read().unwrap().clone();
        let mut neighbours_struct = HashGrid::new(kernel_radius);
        neighbours_struct.insert(&particles);

        let anisotropic_neighbours = if let Some(an_radius) = anisotropic_radius {
            let mut an_grid = HashGrid::new(an_radius);
            an_grid.insert(&particles); // that is sad :(
            an_grid.find_all_neighbours(&particles)
        } else { vec![] };

        Box::new(SimulationFluidSnapshot {
            particles,
            densities,
            neighbours_struct,
            anisotropic_neighbours,
            kernel: CubicSpine::new(self.kernel.radius()),
            mass: self.mass(0),
        })
    }
}

impl Simulation
{
    pub fn new(
        kernel_radius: f32, particle_radius: f32,
        solids: Vec<RigidObject>,
        fluids: Vec<Fluid>,
        camera_position: Vector3<f32>,
        camera_animation: Animation,
        emitters: Vec<Emitter>,
        emitters_animations: Vec<Animation>) -> Simulation
    {
        let volume = 4. * std::f32::consts::PI * particle_radius.powi(3) / 3.;

        Simulation {
            kernel: CubicSpine::new(kernel_radius),
            particle_radius: particle_radius,
            volume: volume,

            rest_density: 1000.0,

            // time step
            cfl_min_time_step: 0.0001,
            cfl_max_time_step: 0.005,
            cfl_factor: 1.0,
            time_step: RwLock::new(0.0001),

            solids,
            debug_solid_collisions: vec![],

            neighbours: Vec::new(),

            density: RwLock::new(Vec::new()),

            len: 0,

            neighbours_struct: HashGrid::new(kernel_radius),
            accelerations: RwLock::new(Vec::new()),
            velocities: RwLock::new(Vec::new()),
            positions: RwLock::new(Vec::new()),
            particles_fluid_type: Vec::new(),
            fluid_types: fluids,

            camera: Camera::new(camera_position),
            camera_animation: camera_animation,

            emitters: emitters,
            emitters_animations: emitters_animations,

            pressure_solver: default_pressure_solver(),
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn clear(&mut self) {
        self.accelerations.write().unwrap().clear();
        self.velocities.write().unwrap().clear();
        self.positions.write().unwrap().clear();
        self.density.write().unwrap().clear();

        // clear pressure & force
        self.len = 0;
    }

    pub fn kernel_radius(&self) -> f32 {
        self.kernel.radius()
    }

    pub fn solid_count(&self) -> usize {
        self.solids.len()
    }

    pub fn solid(&self, i: usize) -> &RigidObject {
        &self.solids[i]
    }

    pub fn debug_get_solid_collisions (&self) -> &Vec<Vector3<f32>> {
        self.debug_solid_collisions.as_ref()
    }

    pub fn get_time_step(&self) -> f32 {
        *self.time_step.read().unwrap()
    }

    pub fn time_step(&self) -> f32 {
        *self.time_step.read().unwrap()
    }

    pub fn add_particle_with_velocity(&mut self, fluid_type: usize, position: Vector3<f32>, velocity: Vector3<f32>)
    {
        for solid in &self.solids {
            if solid.is_particle_inside(&position, self.particle_radius) {
                return;
            }
        }

        self.particles_fluid_type.push(fluid_type);

        self.positions.write().unwrap().push(position);
        self.velocities.write().unwrap().push(velocity);
        self.accelerations.write().unwrap().push(Vector3::zeros());
        self.density.write().unwrap().push(0.0);
        self.len += 1;
    }

    pub fn add_particle(&mut self, fluid_type: usize, x: f32, y: f32, z: f32)
    {
        self.add_particle_with_velocity(fluid_type, Vector3::new(x, y, z), Vector3::zeros());
    }

    pub fn gradient(&self, i: Vector3<f32>, j: Vector3<f32>) -> Vector3<f32> {
        self.kernel.gradient(&(i - j))
    }

    pub fn kernel_apply(&self, i: Vector3<f32>, j: Vector3<f32>) -> f32 {
        self.kernel.apply_on_norm((i - j).norm())
    }

    pub fn particle_radius(&self) -> f32 {
        self.particle_radius
    }

    pub fn volume(&self, _i: usize) -> f32 {
        self.volume
    }

    pub fn mass(&self, i: usize) -> f32 {
        self.fluid_types[self.particles_fluid_type[i]].mass()
    }

    pub fn find_neighbours(&self, x: &Vector3<f32>) -> Vec<usize> {
        self.neighbours_struct.find_neighbours(self.len(), &self.positions.read().unwrap(), *x)
    }

    pub fn neighbours(&self, i: usize) -> Vec<usize> {
        self.neighbours[i].clone()
    }

    pub fn neighbours_count(&self, i: usize) -> usize {
        self.neighbours[i].len()
    }

    pub fn neighbours_count_with_solids(&self, i: usize) -> usize {
        let volume = self.volume(i);

        self.neighbours_count(i) + self.solids_reduce(i, 0, &|_, r, s_volume, _| {
            r + (s_volume / volume) as usize + 1
        })
    }

    pub fn neighbours_reduce<V>(&self, i: usize, value: V, f: &dyn Fn(V, usize, usize) -> V) -> V {
        let mut result = value;

        for j in 0..self.neighbours_count(i) {
            result = f(result, i, self.neighbours[i][j]);
        }

        result
    }

    pub fn solids_reduce<V>(&self, i: usize, value: V, f: &dyn Fn(&RigidObject, V, f32, Vector3<f32>) -> V) -> V {
        let mut result = value;

        for s in &self.solids {
            let volume = s.particle_volume(i);

            if volume > 0.0 {
                result = f(s, result, volume, s.particle_boundary_x(i));
            }
        }

        result
    }

    pub fn neighbours_reduce_v(&self, i: usize, f: &dyn Fn(Vector3<f32>, usize, usize) -> Vector3<f32>) -> Vector3<f32> {
        self.neighbours_reduce(i, Vector3::zeros(), f)
    }

    pub fn neighbours_reduce_f(&self, i: usize, f: &dyn Fn(f32, usize, usize) -> f32) -> f32 {
        self.neighbours_reduce(i, 0.0, f)
    }

    pub fn solids_reduce_v(&self, i: usize, f: &dyn Fn(&RigidObject, Vector3<f32>, f32, Vector3<f32>) -> Vector3<f32>) -> Vector3<f32> {
        self.solids_reduce(i, Vector3::zeros(), f)
    }

    pub fn solids_reduce_f(&self, i: usize, f: &dyn Fn(&RigidObject, f32, f32, Vector3<f32>) -> f32) -> f32 {
        self.solids_reduce(i, 0.0, f)
    }

    pub fn compute_cfl(&self, velocities: &Vec<Vector3<f32>>) -> (f32, f32) {
        let mut v_max: f32 = 0.0;

        for i in 0..self.len() {
            let n = velocities[i].norm_squared();
            v_max = v_max.max(n);
        }

        let v_max = v_max.sqrt();

        let time_step = ((self.cfl_factor * self.particle_radius) / v_max)
            .max(self.cfl_min_time_step)
            .min(self.cfl_max_time_step);

        (v_max, time_step)
    }

    pub fn adapt_cfl(&self) -> f32 {
        let velocities = self.velocities.read().unwrap();

        let (_, time_step) = self.compute_cfl(&velocities);

        *self.time_step.write().unwrap() = time_step;
        time_step
    }

    fn compute_density(&self, i: usize, positions: &Vec<Vector3<f32>>) -> f32 {
        let pos = positions[i];

        let self_dens = self.volume(i) * self.kernel_apply(pos, positions[i]);

        let neighbour_dens = self.neighbours_reduce_f(i, &|density, _, j| {
            density + self.volume(j) * self.kernel_apply(pos, positions[j])
        });

        let solids_dens = self.solids_reduce_f(i, &|_, r, v, x| r + v * self.kernel_apply(pos, x));

        (self_dens + neighbour_dens + solids_dens) * self.rest_density
    }

    pub fn compute_non_pressure_forces(&self) -> f32 {
        let mut accelerations = self.accelerations.write().unwrap();

        accelerations.par_iter_mut().for_each(|v| *v = Vector3::zeros());

        let mut dt = self.time_step();

        for fluid in &self.fluid_types {
            dt = dt.min(fluid.apply_non_pressure_forces(self, &mut accelerations));
        }

        // update velocities with acceleration & dt
        self.velocities.write().unwrap().par_iter_mut().enumerate().for_each(|(i, v)| *v += accelerations[i] * dt);

        *self.time_step.write().unwrap() = dt;
        dt
    }

    fn init(&mut self) {
        self.solids.iter_mut().for_each(|v| v.reset_force_and_torque());
        self.neighbours = self.neighbours_struct.find_all_neighbours(&self.positions.read().unwrap());

        self.init_boundaries();

        let positions = &*self.positions.read().unwrap();

        self.density.write().unwrap().par_iter_mut().enumerate().for_each(|(i, v)| *v = self.compute_density(i, positions));
    }

    fn init_boundaries(&mut self) {
        let pr = self.particle_radius;
        let kr = self.kernel.radius();
        let dt = self.time_step();

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

    pub fn gravity(&self) -> Vector3<f32> {
        Vector3::new(0.0, -9.81, 0.0)
    }

    pub fn init_forces(&mut self) {
        self.init();
        self.fluid_types.par_iter().for_each(|t| t.init_forces(self));
    }

    pub fn compute_vmax(&self) -> f32 {
        self.velocities.read().unwrap()
            .par_iter()
            .map(|v| v.norm_squared())
            .reduce(|| 0.0, |a: f32, b: f32| a.max(b))
            .sqrt()
    }

    pub fn compute_vmean(&self) -> f32 {
        self.velocities.read().unwrap()
            .par_iter()
            .map(|v| v.norm())
            .reduce(|| 0.0, |a: f32, b: f32| a + b) / self.len() as f32
    }

    pub fn update_positions(&mut self) {
        let dt = self.time_step();

        let old = self.positions.read().unwrap().clone();
        self.positions.write().unwrap().par_iter_mut().zip(self.velocities.read().unwrap().par_iter()).for_each(|(p, v)| *p += dt * v);
        self.neighbours_struct.update_particles(&old, &self.positions.read().unwrap());
    }

    pub fn tick(&mut self) -> f32 {
        self.init();

        // fluid handling
        self.pressure_solver.read().unwrap().time_step(self);
        self.update_positions();

        // solids handling
        let dt = self.time_step();

        let gravity = self.gravity();
        self.solids.iter_mut().for_each(|v| v.update(gravity, dt));

        let mut collisions = vec![];

        for i in 0..self.solids.len() {
            for j in i+1..self.solids.len() {
                collisions.par_extend(self.solids[i].collide(&self.solids[j]));
            }
        }

        for i in 0..self.solids.len() {
            self.solids[i].update_vel(dt);
        }

        // update camera
        self.camera.tick(dt, &mut self.camera_animation);

        // emit new particles
        let particles: Vec<(usize, Vector3<f32>, Vector3<f32>)> = self.emitters.par_iter_mut()
            .zip(self.emitters_animations.par_iter_mut())
            .map(|(e, a)| e.tick(dt, a))
            .flatten().collect();

        for (t, p, v) in particles {
            self.add_particle_with_velocity(t, p, v);
        }

        self.debug_solid_collisions = collisions;
        dt
    }
}
