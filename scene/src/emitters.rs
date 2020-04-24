use sph_common::{Emitter, EmitterShape, Animation};
use crate::Scene;

use serde_derive::*;
use nalgebra::Vector3;

#[derive(Debug, Deserialize)]
pub struct EmitterConfig {
    pub shape: EmitterShape,
    pub position: Vector3<f32>,
    pub velocity: f32,

    #[serde(default)]
    pub animation: Animation,
}

impl EmitterConfig {
    pub fn load(&self, scene: &Scene) -> (Emitter, Animation) {
        let radius = scene.config.particle_radius;
        let emitter = Emitter::new(self.position, self.velocity, radius, &self.shape);

        (emitter, self.animation.clone())
    }
}
