use nalgebra::Vector3;
use rayon::prelude::*;

use crate::{Simulation, external_forces::ExternalForce};

pub struct GravityForce(Vector3<f32>);

impl GravityForce {
    pub fn new(f: Vector3<f32>) -> Box<GravityForce> {
        Box::new(GravityForce(f))
    }
}

impl ExternalForce for GravityForce {
    fn init(&mut self, _: &Simulation) { }

    fn compute_acceleration(&self, _sim: &Simulation, accelerations: &mut Vec<Vector3<f32>>) -> Option<f32> {
        accelerations.par_iter_mut().for_each(|v| *v += self.0);
        None
    }
}
