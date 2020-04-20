use nalgebra::Vector3;
use rayon::prelude::*;

use crate::{DFSPH, external_forces::ExternalForce};

pub struct GravityForce(Vector3<f32>);

impl GravityForce {
    pub fn new(f: Vector3<f32>) -> Box<GravityForce> {
        Box::new(GravityForce(f))
    }
}

impl ExternalForce for GravityForce {
    fn compute_acceleration(&self, _sim: &DFSPH, accelerations: &mut Vec<Vector3<f32>>) {
        accelerations.par_iter_mut().for_each(|v| *v += self.0)
    }
}
