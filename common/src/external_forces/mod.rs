mod gravity;
mod surfacetension;
mod basic_viscosity;
mod high_viscosity;
mod vorticity;

use serde_derive::*;

use nalgebra::Vector3;
use crate::DFSPH;

#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
pub enum ViscosityType {
    #[serde(rename = "basic")]
    Basic {
        coefficient: f32,
        surface_coefficient: f32,
    },
    #[serde(rename = "weiler2018")]
    Weiler2018 {
        coefficient: f32,
        surface_coefficient: f32,
        max_iteration: usize,
        tolerance: f32,
    }
}

impl Default for ViscosityType {
    fn default() -> ViscosityType {
        ViscosityType::Basic {
            coefficient: 0.01,
            surface_coefficient: 0.005,
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct VorticityConfig {
    inertia_inverse: f32,
    viscosity_omega: f32,
    vorticity_coefficient: f32,
}

impl Default for VorticityConfig {
    fn default() -> VorticityConfig {
        VorticityConfig {
            inertia_inverse: 0.5,
            viscosity_omega: 0.1,
            vorticity_coefficient: 0.01,
        }
    }
}

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

    pub fn viscosity(&mut self, viscosity: &ViscosityType) -> &mut ExternalForces {
        let force: Box<dyn ExternalForce + Sync + Send> = match viscosity {
            ViscosityType::Basic { coefficient, surface_coefficient } => basic_viscosity::BasicViscosityForce::new(*coefficient, *surface_coefficient),
            ViscosityType::Weiler2018{ coefficient, surface_coefficient, max_iteration, tolerance} => high_viscosity::ViscosityWeiler2018Force::new(*coefficient, *surface_coefficient, *max_iteration, *tolerance),
        };

        self.add(force)
    }

    pub fn vorticity(&mut self, config: &VorticityConfig) -> &mut ExternalForces {
        self.add(vorticity::VorticityForce::new(
            config.vorticity_coefficient,
            config.inertia_inverse,
            config.viscosity_omega,
        ))
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
