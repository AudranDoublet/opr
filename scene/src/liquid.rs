use serde_derive::*;
use nalgebra::Vector3;

use sph::{Simulation, Animation};

use crate::{Scene, Solid};

fn default_density() -> f32 {
    2.
}

#[derive(Debug, Deserialize)]
pub enum FixedStrategy {
    Empty,
    Lowest,
}

impl Default for FixedStrategy {
    fn default() -> FixedStrategy {
        FixedStrategy::Empty
    }
}

impl FixedStrategy {
    pub fn fixed(&self, points: Vec<Vector3<f32>>) -> Vec<(bool, Vector3<f32>)> {
        match self {
            FixedStrategy::Empty    => points.iter().map(|v| (false, *v)).collect(),
            FixedStrategy::Lowest   => {
                let y_min = points.iter().fold(std::f32::INFINITY, |a, b| a.min(b.y));

                points.iter().map(|v| ((v.y - y_min).abs() < 0.01, *v)).collect()
            },
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
pub enum LiquidZone {
    #[serde(rename = "block")]
    Block {
        from: Vector3<f32>,
        to: Vector3<f32>,
        #[serde(default = "default_density")]
        density: f32,

        #[serde(default)]
        fluid_type: Option<String>,

        #[serde(default)]
        fixed_strategy: FixedStrategy,
    },
    #[serde(rename = "mesh")]
    Mesh {
        #[serde(default)]
        slice: bool,
        mesh: String,
        scale: [f32; 3],

        position: [f32; 3],
        rotation_axis: [f32; 3],
        rotation_angle: f32,

        resolution: [u32; 3],

        #[serde(default = "default_density")]
        density: f32,

        #[serde(default)]
        fluid_type: Option<String>,

        #[serde(default)]
        fixed_strategy: FixedStrategy,
    },
}

impl LiquidZone
{
    fn fluid_type(&self, config: &Scene) -> usize {
        let map = config.load_fluids_map();

        let name = match self {
            LiquidZone::Block { fluid_type, .. } => fluid_type,
            LiquidZone::Mesh  { fluid_type, .. } => fluid_type,
        };

        if let Some(n) = &name {
            map[n]
        } else {
            0
        }
    }

    fn fixed_strategy(&self) -> &FixedStrategy {
        match self {
            LiquidZone::Block { fixed_strategy, .. } => fixed_strategy,
            LiquidZone::Mesh  { fixed_strategy, .. } => fixed_strategy,
        }
    }

    pub fn create_particles(&self, config: &Scene, scene: &mut Simulation) -> Result<(), Box<dyn std::error::Error>>
    {
        let fluid_type = self.fluid_type(config);
        let fixed_strategy = self.fixed_strategy();

        let positions = match self {
            LiquidZone::Block { from, to, density, .. } => {
                let radius = scene.particle_radius() * density;
                let step_count = (to - from) / radius;

                let mut particles = Vec::new();

                for z in 0..step_count.z as usize {
                    let z = z as f32 * radius;
                    for y in 0..step_count.y as usize {
                        let y = y as f32 * radius;
                        for x in 0..step_count.x as usize {
                            let x = x as f32 * radius;
                            particles.push(Vector3::new(from.x + x, from.y + y, from.z + z));
                        }
                    }
                }

                particles
            },
            LiquidZone::Mesh { mesh, scale, position, rotation_axis, rotation_angle, resolution, slice, density, .. } => {
                let solid = Solid {
                    animation: Animation::Blank,
                    mesh: mesh.to_string(),
                    mesh_invert: false,
                    scale: *scale,
                    position: *position,
                    rotation_axis: *rotation_axis,
                    rotation_angle: *rotation_angle,
                    density: 1000.,
                    resolution: *resolution,
                    display: true,
                    dynamic: false,
                    particle_size: 1000.0,
                    slice: *slice,
                };

                let solid = solid.load(config)?;
                let position = solid.position();

                solid.to_particles(scene.kernel_radius(), scene.particle_radius() * density)
                     .iter().map(|v| v + position)
                     .collect()
            }
        };

        let count = positions.len();

        for (fixed, position) in fixed_strategy.fixed(positions) {
            scene.add_particle_with_velocity(fixed, fluid_type, position, Vector3::zeros());
        }

        println!("{} particles added", count);
        Ok(())
    }
}
