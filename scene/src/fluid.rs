use nalgebra::Vector3;

use serde_derive::*;

use sph::external_forces::{ExternalForces, ViscosityType, VorticityConfig, DragConfig, ElasticityConfig};
use sph::Fluid;

fn default_debug_color() -> Vector3<f32> {
    Vector3::new(0.0, 0.0, 1.0)
}

fn default_surface_tension() -> f32 {
    0.05
}

fn default_surface_adhesion() -> f32 {
    0.01
}

#[derive(Debug, Deserialize)]
pub struct FluidConfiguration {
    density: f32,
    #[serde(default = "default_debug_color")]
    pub debug_color: Vector3<f32>,
    #[serde(default = "default_surface_tension")]
    pub surface_tension: f32,
    #[serde(default = "default_surface_adhesion")]
    pub surface_adhesion: f32,
    #[serde(default)]
    pub viscosity: ViscosityType,
    #[serde(default)]
    pub vorticity: VorticityConfig,
    #[serde(default)]
    pub elasticity: Option<ElasticityConfig>,
    #[serde(default)]
    pub drag: DragConfig,
}

impl FluidConfiguration {
    pub fn create(&self, id: usize, volume: f32, gravity: Vector3<f32>, kernel_radius: f32) -> Fluid {
        let mut forces = ExternalForces::new();

        forces.gravity(gravity)
              .surface_tension(kernel_radius, self.surface_tension, self.surface_adhesion)
              .viscosity(&self.viscosity)
              .vorticity(&self.vorticity)
              .drag(&self.drag)
              .elasticity(&self.elasticity);

        Fluid::new(id, volume, self.density, forces, self.debug_color)
    }
}
