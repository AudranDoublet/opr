use nalgebra::Vector3;

use crate::{DFSPH, external_forces::ExternalForce};

pub struct GravityForce(Vector3<f32>);

impl GravityForce {
    pub fn new(f: Vector3<f32>) -> Box<GravityForce> {
        Box::new(GravityForce(f))
    }
}

impl ExternalForce for GravityForce {
    fn compute_acceleration(&self, _sim: &DFSPH, _i: usize) -> Vector3<f32> {
        self.0
    }
}
