use nalgebra::Vector3;
use serde_derive::*;
use sph::external_forces::{DragConfig, ElasticityConfig, ExternalForces, ViscosityType, VorticityConfig};
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
pub struct FluidConfigurationSimulation {
    density: f32,
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

#[derive(Debug, Deserialize)]
pub struct ConfigurationAnisotropication {
    #[serde(default = "anisotropication_conf_default_smoothness")]
    pub smoothness: f32,
    #[serde(default = "anisotropication_conf_default_nb_neighbours")]
    pub min_nb_neighbours: usize,
    #[serde(default = "anisotropication_conf_default_kr")]
    pub kr: f32,
    #[serde(default = "anisotropication_conf_default_ks")]
    pub ks: f32,
    #[serde(default = "anisotropication_conf_default_kn")]
    pub kn: f32,
}

fn anisotropication_conf_default_smoothness() -> f32 { 0.9 }
fn anisotropication_conf_default_nb_neighbours() -> usize { 5 }
fn anisotropication_conf_default_kr() -> f32 { 8. }
fn anisotropication_conf_default_ks() -> f32 { 1400. }
fn anisotropication_conf_default_kn() -> f32 { 0.5 }

impl Default for ConfigurationAnisotropication {
    fn default() -> Self {
        ConfigurationAnisotropication {
            smoothness: anisotropication_conf_default_smoothness(),
            min_nb_neighbours: anisotropication_conf_default_nb_neighbours(),
            kr: anisotropication_conf_default_kr(),
            ks: anisotropication_conf_default_ks(),
            kn: anisotropication_conf_default_kn(),
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct FluidConfigurationMeshing
{
    #[serde(default = "meshing_conf_default_iso_value")]
    pub iso_value: f32,
    #[serde(default = "meshing_conf_default_cube_size")]
    pub cube_size: f32,
    #[serde(default = "meshing_conf_default_enable_interpolation")]
    pub enable_interpolation: bool,
    #[serde(default = "meshing_conf_default_enable_anisotropication")]
    pub enable_anisotropication: bool,
    #[serde(default)]
    pub anisotropication_config: ConfigurationAnisotropication,
}

fn meshing_conf_default_iso_value() -> f32 { 0.005 }
fn meshing_conf_default_cube_size() -> f32 { 0.04 }
fn meshing_conf_default_enable_interpolation() -> bool { true }
fn meshing_conf_default_enable_anisotropication() -> bool { true }

impl Default for FluidConfigurationMeshing {
    fn default() -> Self {
        FluidConfigurationMeshing {
            iso_value: meshing_conf_default_iso_value(),
            cube_size: meshing_conf_default_cube_size(),
            enable_interpolation: meshing_conf_default_enable_interpolation(),
            enable_anisotropication: meshing_conf_default_enable_anisotropication(),
            anisotropication_config: Default::default()
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct FluidConfiguration {
    #[serde(default = "default_debug_color")]
    pub debug_color: Vector3<f32>,
    #[serde(default)]
    pub material: Option<String>,
    #[serde(default)]
    pub track_with_bubbler: bool,
    pub simulation: FluidConfigurationSimulation,
    pub meshing: FluidConfigurationMeshing,
}

impl FluidConfiguration {
    pub fn create(&self, id: usize, volume: f32, gravity: Vector3<f32>, kernel_radius: f32) -> Fluid {
        let mut forces = ExternalForces::new();

        let conf_simulation = &self.simulation;

        forces.gravity(gravity)
            .surface_tension(kernel_radius, conf_simulation.surface_tension, conf_simulation.surface_adhesion)
            .viscosity(&conf_simulation.viscosity)
            .vorticity(&conf_simulation.vorticity)
            .drag(&conf_simulation.drag)
            .elasticity(&conf_simulation.elasticity);

        Fluid::new(id, volume, conf_simulation.density, forces, self.debug_color)
    }
}
