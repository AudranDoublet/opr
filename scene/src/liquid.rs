use serde_derive::*;
use nalgebra::Vector3;

use sph_common::DFSPH;

use crate::{Scene, Solid};

#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
pub enum LiquidZone {
    #[serde(rename = "block")]
    Block {
        from: Vector3<f32>,
        to: Vector3<f32>,
    },
    #[serde(rename = "mesh")]
    Mesh {
        mesh: String,
        scale: [f32; 3],

        position: [f32; 3],
        rotation_axis: [f32; 3],
        rotation_angle: f32,

        resolution: [u32; 3],
    },
}

impl LiquidZone
{
    pub fn create_particles(&self, config: &Scene, scene: &mut DFSPH) -> Result<(), Box<dyn std::error::Error>>
    {
        let count = match self {
            LiquidZone::Block { from, to } => {
                let radius = scene.particle_radius() * 2.;
                let step_count = (to - from) / radius;

                let mut count = 0;

                for z in 0..step_count.z as usize {
                    let z = z as f32 * radius;
                    for y in 0..step_count.y as usize {
                        let y = y as f32 * radius;
                        for x in 0..step_count.x as usize {
                            let x = x as f32 * radius;
                            scene.add_particle(from.x + x, from.y + y, from.z + z);

                            count += 1;
                        }
                    }
                }

                count
            },
            LiquidZone::Mesh { mesh, scale, position, rotation_axis, rotation_angle, resolution } => {
                let solid = Solid {
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
                    slice: false,
                };

                let solid = solid.load(config)?;
                let position = solid.position();

                let mut count = 0;

                for particle in &solid.to_particles(scene.particle_radius() * 2.) {
                    let pos = particle + position;

                    scene.add_particle(pos.x, pos.y, pos.z);
                    count += 1;
                }

                count
            }
        };

        println!("{} particles added", count);
        Ok(())
    }
}
