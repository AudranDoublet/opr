use crate::Simulation;

pub trait PressureSolver {
    fn time_step(&self, sim: &Simulation);
}

mod dfsph;
pub use dfsph::*;
