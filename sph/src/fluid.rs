use std::sync::RwLock;

use rayon::iter::{ParallelIterator, IndexedParallelIterator};
use rayon::prelude::*;

use nalgebra::Vector3;

use crate::Simulation;
use crate::external_forces::ExternalForces;

use serde_derive::*;

#[derive(Serialize, Deserialize)]
pub struct Fluid {
    fluid_type: usize,
    rest_density: f32,
    mass: f32,

    #[serde(skip_serializing, skip_deserializing)]
    correspondances: Vec<usize>,
    #[serde(skip_serializing, skip_deserializing)]
    external_forces: RwLock<ExternalForces>,

    debug_color: Vector3<f32>,
}

impl Fluid {
    pub fn new(id: usize, volume: f32, density: f32, forces: ExternalForces, debug_color: Vector3<f32>) -> Fluid {
        Fluid {
            fluid_type: id,
            rest_density: density,
            mass: volume * density,
            external_forces: RwLock::new(forces),
            correspondances: Vec::new(),
            debug_color: debug_color,
        }
    }

    #[inline]
    pub fn idx(&self) -> usize {
        self.fluid_type
    }

    #[inline]
    pub fn rest_density(&self) -> f32 {
        self.rest_density
    }

    #[inline]
    pub fn mass(&self) -> f32 {
        self.mass
    }

    #[inline]
    pub fn debug_color(&self) -> Vector3<f32> {
        self.debug_color
    }

    pub fn correspondance(&self, i: usize) -> usize {
        self.correspondances[i]
    }

    pub fn filter<'a, T: 'a + Send + Sync>(&self, fixed: bool, sim: &'a Simulation, iter: impl ParallelIterator<Item=&'a T> + IndexedParallelIterator) -> impl ParallelIterator<Item=(usize, &'a T)> {
        let idx = self.idx();

        iter.enumerate()
            .filter(move |(i, _)| idx == sim.particles_fluid_type[*i] && (fixed || !sim.fixed[*i]))
    }

    pub fn filter_range<'a>(&self, fixed: bool, sim: &'a Simulation) -> impl ParallelIterator<Item=usize>  + 'a {
        let idx = self.idx();

        (0..sim.len())
            .into_par_iter()
            .filter(move |i| idx == sim.particles_fluid_type[*i] && (fixed || !sim.fixed[*i]))
    }

    pub fn filter_m<'a, T: 'a + Send + Sync>(&self, fixed: bool, sim: &'a Simulation, iter: impl ParallelIterator<Item=&'a mut T> + IndexedParallelIterator) -> impl ParallelIterator<Item=(usize, &'a mut T)> {
        let idx = self.idx();

        iter.enumerate()
            .filter(move |(i, _)| idx == sim.particles_fluid_type[*i] && (fixed || !sim.fixed[*i]))
    }

    pub fn filter_mt<'a, T: 'a + Send + Sync, U: 'a + Send + Sync>(&self, fixed: bool, sim: &'a Simulation, iter: impl ParallelIterator<Item=(&'a mut T, &'a mut U)> + IndexedParallelIterator) -> impl ParallelIterator<Item=(usize, (&'a mut T, &'a mut U))> {
        let idx = self.idx();

        iter.enumerate()
            .filter(move |(i, _)| idx == sim.particles_fluid_type[*i] && (fixed || !sim.fixed[*i]))
    }

    pub fn init_forces(&self, sim: &Simulation) {
        self.external_forces
            .write()
            .unwrap()
            .init(self, sim);
    }

    pub fn compute_correspondance_table(&mut self, v: &Vec<usize>) {
        if self.correspondances.len() == v.len() {
            return;
        }

        self.correspondances = vec![0; v.len()];

        let mut curr = 0;

        for i in 0..v.len() {
            if v[i] == self.idx() {
                self.correspondances[i] = curr;
                curr += 1;
            }
        }
    }

    pub fn apply_non_pressure_forces(&self, sim: &Simulation, accelerations: &mut Vec<Vector3<f32>>) -> f32 {
        self.external_forces
            .write()
            .unwrap()
            .apply(self, sim, accelerations)
    }
}
