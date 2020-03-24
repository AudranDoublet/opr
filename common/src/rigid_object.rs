use nalgebra::Vector3;

use crate::{DiscreteGrid, Particle};

pub struct RigidObject
{
    position: Vector3<f32>,
    scale: Vector3<f32>,
    grid: DiscreteGrid,
}

impl RigidObject
{
    pub fn new(grid: DiscreteGrid) -> RigidObject {
        RigidObject {
            position: Vector3::zeros(),
            scale: Vector3::zeros(),
            grid: grid,
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

    pub fn position_in_mesh_space(&self, p: Vector3<f32>) -> Vector3<f32>
    {
        p - self.position //FIXME rotation
    }

    pub fn compute_volume_and_boundary_x(&self, particle: &mut Particle, particle_radius: f32, kernel_radius: f32, dt: f32) -> (f32, Vector3<f32>)
    {
        let x = self.position_in_mesh_space(particle.position);

        let mut volume = 0.0;
        let mut boundary_x = Vector3::zeros();

        let (dist, normal) = self.grid.interpolate_or(0, x, true);

        if dist > 0.1 * particle_radius && dist < kernel_radius {
            if let Some((v, _)) = self.grid.interpolate(1, x, false) {
                if v > 1e-5 { // != 0
                    volume = v;

                    //FIXME unrotate normal

                    if normal.norm() > 1e-5 { // != 0
                        boundary_x = particle.position - dist*normal.normalize();
                    }
                } else {
                    //FIXME unrotate normal

                    if normal.norm() > 1e-5 { // != 0
                        // particle is 'in' surface, update position and velocity in order to fix
                        // them
                        let normal = normal.normalize();

                        let d = (-dist).min(50. * particle_radius * dt);
                        particle.position += d * normal;
                        particle.velocity += (0.05 - particle.velocity.dot(&normal)) * normal;
                    }
                }
            }
        }

        (volume, boundary_x)
    }
}
