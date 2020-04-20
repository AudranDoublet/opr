mod gravity;
mod surfacetension;

use nalgebra::Vector3;
use crate::DFSPH;

pub trait ExternalForce
{
    fn compute_acceleration(&self, sim: &DFSPH, accelerations: &mut Vec<Vector3<f32>>);
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

    pub fn surface_tension(&mut self, kernel_radius: f32, surface_tension: f32, surface_adhesion: f32) -> &mut ExternalForces {
        self.add(surfacetension::SurfaceTensionForce::new(kernel_radius, surface_tension, surface_adhesion))
    }

    pub fn add(&mut self, force: Box<dyn ExternalForce + Sync + Send>) -> &mut ExternalForces {
        self.forces.push(force);
        self
    }

    pub fn apply(&self, sim: &DFSPH, accelerations: &mut Vec<Vector3<f32>>) {
        for v in &self.forces {
            v.compute_acceleration(sim, accelerations);
        }
    }
}

impl Default for ExternalForces {
    fn default() -> ExternalForces {
        ExternalForces::new()
    }
}
