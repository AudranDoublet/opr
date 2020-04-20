mod gravity;

use nalgebra::Vector3;
use crate::DFSPH;

pub trait ExternalForce
{
    fn compute_acceleration(&self, sim: &DFSPH, i: usize) -> Vector3<f32>;
}

pub struct ExternalForces
{
    forces: Vec<Box<dyn ExternalForce + Sync + Send>>,
}

impl ExternalForces {
    pub fn new() -> ExternalForces {
        ExternalForces {
            forces: Vec::new(),
        }
    }

    pub fn gravity(&mut self, intensity: Vector3<f32>) -> &mut ExternalForces {
        self.add(gravity::GravityForce::new(intensity))
    }

    pub fn add(&mut self, force: Box<dyn ExternalForce + Sync + Send>) -> &mut ExternalForces {
        self.forces.push(force);
        self
    }

    pub fn apply(&self, sim: &DFSPH, i: usize) -> Vector3<f32> {
        self.forces.iter()
            .fold(Vector3::zeros(), |a, b| a + b.compute_acceleration(sim, i))
    }
}

impl Default for ExternalForces {
    fn default() -> ExternalForces {
        ExternalForces::new()
    }
}
