use std::sync::RwLock;

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
    external_forces: RwLock<ExternalForces>,
}

impl Fluid {
    pub fn new(id: usize, volume: f32, density: f32, forces: ExternalForces) -> Fluid {
        Fluid {
            fluid_type: id,
            rest_density: density,
            mass: volume * density,
            external_forces: RwLock::new(forces),
        }
    }

    #[inline]
    pub fn rest_density(&self) -> f32 {
        self.rest_density
    }

    #[inline]
    pub fn mass(&self) -> f32 {
        self.mass
    }

    pub fn init_forces(&self, sim: &Simulation) {
        self.external_forces
            .write()
            .unwrap()
            .init(sim); // FIXME fluid_type
    }

    pub fn apply_non_pressure_forces(&self, sim: &Simulation, accelerations: &mut Vec<Vector3<f32>>) -> f32 {
        self.external_forces
            .write()
            .unwrap()
            .apply(sim, accelerations) // FIXME fluid_type
    }
}
