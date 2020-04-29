use std::sync::RwLock;

use rayon::iter::{ParallelIterator, IndexedParallelIterator};
use rayon::prelude::*;

use nalgebra::Vector3;

use crate::Simulation;
use crate::external_forces::ExternalForces;

pub struct Fluid {
    fluid_type: usize,
    rest_density: f32,
    mass: f32,

    correspondances: Vec<usize>,
    external_forces: RwLock<ExternalForces>,
}

impl Fluid {
    pub fn new(id: usize, volume: f32, density: f32, forces: ExternalForces) -> Fluid {
        Fluid {
            fluid_type: id,
            rest_density: density,
            mass: volume * density,
            external_forces: RwLock::new(forces),
            correspondances: Vec::new(),
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

    pub fn correspondance(&self, i: usize) -> usize {
        self.correspondances[i]
    }

    pub fn filter<'a, T: 'a + Send + Sync>(&self, sim: &'a Simulation, iter: impl ParallelIterator<Item=&'a T> + IndexedParallelIterator) -> impl ParallelIterator<Item=(usize, &'a T)> {
        let idx = self.idx();

        iter.enumerate()
            .filter(move |(i, _)| idx == sim.particles_fluid_type[*i])
    }

    pub fn filter_range<'a>(&self, sim: &'a Simulation) -> impl ParallelIterator<Item=usize>  + 'a {
        let idx = self.idx();

        (0..sim.len())
            .into_par_iter()
            .filter(move |i| idx == sim.particles_fluid_type[*i])
    }

    pub fn filter_m<'a, T: 'a + Send + Sync>(&self, sim: &'a Simulation, iter: impl ParallelIterator<Item=&'a mut T> + IndexedParallelIterator) -> impl ParallelIterator<Item=(usize, &'a mut T)> {
        let idx = self.idx();

        iter.enumerate()
            .filter(move |(i, _)| idx == sim.particles_fluid_type[*i])
    }

    pub fn filter_mt<'a, T: 'a + Send + Sync, U: 'a + Send + Sync>(&self, sim: &'a Simulation, iter: impl ParallelIterator<Item=(&'a mut T, &'a mut U)> + IndexedParallelIterator) -> impl ParallelIterator<Item=(usize, (&'a mut T, &'a mut U))> {
        let idx = self.idx();

        iter.enumerate()
            .filter(move |(i, _)| idx == sim.particles_fluid_type[*i])
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
