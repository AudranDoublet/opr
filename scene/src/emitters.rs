use sph::{Emitter, Animation};
use crate::{Scene, LiquidZone};

use serde_derive::*;
use nalgebra::Vector3;

#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
pub enum EmitterShape {
    #[serde(rename = "ellipse")]
    Ellipse {
        x_radius: f32,
        y_radius: f32,
    },
    #[serde(rename = "rectangle")]
    Rectangle {
        width: f32,
        height: f32,
    },
    #[serde(rename = "block")]
    LiquidBlock {
        block: LiquidZone,
    }
}

impl EmitterShape {
    pub fn is_shot(&self) -> bool {
        match self {
            EmitterShape::LiquidBlock { .. } => true,
            _ => false,
        }
    }

    pub fn generate_particles(&self, scene: &Scene) -> Result<Vec<Vector3<f32>>, Box<dyn std::error::Error>> {
        let particle_radius = scene.config.particle_radius * 2.;

        match self {
            EmitterShape::Ellipse { x_radius, y_radius } => {
                let xr = x_radius.powi(2);
                let yr = y_radius.powi(2);

                let rectangle = EmitterShape::Rectangle { width: *x_radius, height: *y_radius };

                Ok(rectangle.generate_particles(scene)?
                         .iter()
                         .filter(|p| p.x*p.x / xr + p.y*p.y / yr <= 1.0)
                         .map(|p| *p)
                         .collect())
            },
            EmitterShape::Rectangle { width, height } => {
                let x0 = -width/2.;
                let y0 = -height/2.;
                let step_x = (width / particle_radius) as usize;
                let step_y = (height / particle_radius) as usize;

                let mut res = Vec::new();
                for y in 0..step_y {
                    for x in 0..step_x {
                        res.push(Vector3::new(
                                x as f32 * particle_radius + x0,
                                y as f32 * particle_radius + y0,
                                0.0,
                        ));
                    }
                }

                Ok(res)
            },
            EmitterShape::LiquidBlock { block } => {
                block.particle_list(scene)
            }
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct EmitterConfig {
    pub shape: EmitterShape,
    pub position: Vector3<f32>,
    pub velocity: f32,

    #[serde(default)]
    pub fluid_type: Option<String>,

    #[serde(default)]
    pub animation: Animation,
}

impl EmitterConfig {
    fn fluid_type(&self, config: &Scene) -> usize {
        let map = config.load_fluids_map();

        if let Some(n) = &self.fluid_type {
            map[n]
        } else {
            0
        }
    }

    pub fn load(&self, scene: &Scene) -> Result<(Emitter, Animation), Box<dyn std::error::Error>> {
        let particles = self.shape.generate_particles(scene)?;

        let emitter = Emitter::new(self.fluid_type(scene), self.position,
                    self.velocity, scene.config.particle_radius,
                    particles, self.shape.is_shot());

        Ok((emitter, self.animation.clone()))
    }
}
