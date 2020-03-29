use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::{DiscreteGrid};

#[derive(Serialize, Deserialize, Debug)]
pub struct RigidObject
{
    position: Vector3<f32>,
    scale: Vector3<f32>,
    #[serde(skip_serializing, skip_deserializing)]
    grid: DiscreteGrid,
    volume: Vec<f32>,
    #[serde(skip_serializing)]
    #[serde(skip_deserializing)]
    boundary_x: Vec<Vector3<f32>>,
}

impl RigidObject
{
    pub fn new(grid: DiscreteGrid) -> RigidObject {
        RigidObject {
            position: Vector3::zeros(),
            scale: Vector3::zeros(),
            grid,
            volume: Vec::new(),
            boundary_x: Vec::new(),
        }
    }

    pub fn set_position(&mut self, position: Vector3<f32>)
    {
        self.position = position
    }

    pub fn set_scale(&mut self, scale: Vector3<f32>)
    {
        self.scale = scale
    }

    pub fn particle_volume(&self, i: usize) -> f32 {
        self.volume[i]
    }

    pub fn particle_boundary_x(&self, i: usize) -> Vector3<f32> {
        self.boundary_x[i]
    }

    pub fn position_in_mesh_space(&self, p: Vector3<f32>) -> Vector3<f32>
    {
        p - self.position//.component_mul(&self.scale) //FIXME rotation
    }

    pub fn set_volume_and_boundary_x(&mut self, volume: Vec<f32>, boundary_x: Vec<Vector3<f32>>)
    {
        self.volume = volume;
        self.boundary_x = boundary_x;
    }

    pub fn compute_volume_and_boundary_x(&self, position: &mut Vector3<f32>, velocity: &mut Vector3<f32>, particle_radius: f32, kernel_radius: f32, dt: f32) -> (f32, Vector3<f32>)
    {
        let x = self.position_in_mesh_space(*position);

        let mut volume = 0.0;
        let mut boundary_x = Vector3::zeros();

        let (dist, normal) = self.grid.interpolate_or(0, x, true);

        if dist > 0.1 * particle_radius && dist < kernel_radius {
            if let Some((v, _)) = self.grid.interpolate(1, x, false) {
                if v > 1e-5 { // != 0
                    volume = v;

                    //FIXME unrotate normal

                    if normal.norm() > 1e-5 { // != 0
                        boundary_x = *position - dist * normal.normalize();
                    }
                }
            }
        } else if dist < 0.1 * particle_radius {
            //FIXME unrotate normal

            if normal.norm() > 1e-5 { // != 0
                // particle is 'in' surface, update position and velocity in order to fix
                // them
                let normal = normal.normalize();

                let d = (-dist).min(50. * particle_radius * dt);
                *position += d * normal;
                *velocity += (0.05 - velocity.dot(&normal)) * normal;
            }
        }

        (volume, boundary_x)
    }
}
