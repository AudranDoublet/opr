mod gravity;
mod surfacetension;
mod basic_viscosity;
mod high_viscosity;
mod vorticity;
mod drag;
mod elasticity;

use serde_derive::*;

use nalgebra::Vector3;

use crate::Fluid;
use crate::Simulation;

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

#[derive(Debug, Deserialize)]
pub struct DragConfig {
    drag_coefficient: f32,
    air_velocity: Vector3<f32>,
}

impl Default for DragConfig {
    fn default() -> DragConfig {
        DragConfig {
            drag_coefficient: 1.0,
            air_velocity: Vector3::zeros(),
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct ElasticityConfig {
    young_modulus: f32,
    poisson_ratio: f32,
    alpha: f32,
    max_iteration: usize,
    tolerance: f32,
}

pub trait ExternalForce
{
    fn compute_acceleration(&self, fluid: &Fluid, sim: &Simulation, accelerations: &mut Vec<Vector3<f32>>) -> Option<f32>;

    fn init(&mut self, fluid: &Fluid, sim: &Simulation);
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

    pub fn drag(&mut self, config: &DragConfig) -> &mut ExternalForces {
        self.add(drag::DragForce::new(
            config.air_velocity,
            config.drag_coefficient,
        ))
    }

    pub fn elasticity(&mut self, config: &Option<ElasticityConfig>) -> &mut ExternalForces {
        if let Some(c) = config {
            self.add(elasticity::ElasticityForce::new(
                c.young_modulus,
                c.poisson_ratio,
                c.alpha,
                c.max_iteration,
                c.tolerance,
            ))
        } else {
            self
        }
    }

    pub fn add(&mut self, force: Box<dyn ExternalForce + Sync + Send>) -> &mut ExternalForces {
        self.forces.push(force);
        self
    }

    pub fn init(&mut self, fluid: &Fluid, sim: &Simulation) {
        for v in &mut self.forces {
            v.init(fluid, sim);
        }
    }

    pub fn apply(&self, fluid: &Fluid, sim: &Simulation, accelerations: &mut Vec<Vector3<f32>>) -> f32 {
        let mut dt = sim.time_step();

        for v in &self.forces {
            if let Some(v) = v.compute_acceleration(fluid, sim, accelerations) {
                dt = v;
            }
        }

        dt
    }
}

impl Default for ExternalForces {
    fn default() -> ExternalForces {
        ExternalForces::new()
    }
}
