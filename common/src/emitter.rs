use nalgebra::{Vector3, Quaternion, UnitQuaternion};

use std::sync::RwLock;

use crate::{Animation, VariableType, AnimationHandler};

use serde_derive::*;

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
    }
}

impl EmitterShape {
    pub fn generate_particles(&self, particle_radius: f32) -> Vec<Vector3<f32>> {
        match self {
            EmitterShape::Ellipse { x_radius, y_radius } => {
                let xr = x_radius.powi(2);
                let yr = y_radius.powi(2);

                let rectangle = EmitterShape::Rectangle { width: *x_radius, height: *y_radius };
                rectangle.generate_particles(particle_radius)
                         .iter()
                         .filter(|p| p.x*p.x / xr + p.y*p.y / yr <= 1.0)
                         .map(|p| *p)
                         .collect()
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

                res
            }
        }
    }
}

pub struct Emitter {
    particle_velocity: f32,

    position: Vector3<f32>,
    velocity: Vector3<f32>,
    angular_velocity: Vector3<f32>,
    rotation: Quaternion<f32>,

    acceleration: Vector3<f32>,
    angular_acceleration: Vector3<f32>,

    particles: Vec<Vector3<f32>>,
    time_between_emits: f32,
    next_emit: RwLock<f32>,
}

impl Emitter {
    pub fn new(position: Vector3<f32>, particle_velocity: f32,
                particle_radius: f32, shape: &EmitterShape) -> Emitter {
        Emitter {
            particle_velocity: particle_velocity,
            position: position,
            velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            rotation: Quaternion::identity(),
            acceleration: Vector3::zeros(),
            angular_acceleration: Vector3::zeros(),
            particles: shape.generate_particles(particle_radius * 2.),

            next_emit: RwLock::new(0.0),
            time_between_emits: particle_radius * 2. / particle_velocity,
        }
    }

    pub fn emit(&self) -> Vec<(Vector3<f32>, Vector3<f32>)> {
        let rotation = UnitQuaternion::from_quaternion(self.rotation);
        let pvelocity = (rotation * Vector3::z()) * self.particle_velocity;

        let mut res = Vec::new();

        for particle in &self.particles {
            res.push((
                rotation * particle + self.position,
                pvelocity,
            ));
        }

        res
    }

    pub fn tick(&mut self, dt: f32, animation: &mut Animation) -> Vec<(Vector3<f32>, Vector3<f32>)> {
        animation.animate(self, dt, false);

        self.velocity += self.acceleration * dt;
        self.position += self.velocity * dt;

        self.angular_velocity += self.angular_acceleration * dt;

        let q = Quaternion::new(0.0, self.angular_velocity.x, self.angular_velocity.y, self.angular_velocity.z);

        self.rotation += (dt / 2.) * (q * self.rotation);
        self.rotation.normalize_mut();

        let mut next_emit = self.next_emit.write().unwrap();

        if *next_emit - dt <= 0.0 {
            *next_emit = self.time_between_emits;
            self.emit()
        } else {
            *next_emit -= dt;
            Vec::new()
        }
    }
}

impl AnimationHandler for Emitter {
    fn get_variable(&self, variable: &VariableType) -> Vector3<f32> {
        match variable {
            VariableType::AngularVelocity => self.angular_velocity,
            VariableType::Velocity => self.velocity,
            VariableType::AngularAcceleration => self.angular_acceleration,
            VariableType::Acceleration => self.acceleration,
            VariableType::Position => self.position,
            VariableType::Rotation => {
                let (x, y, z) = UnitQuaternion::from_quaternion(self.rotation).euler_angles();
                Vector3::new(x, y, z)
            },
        }
    }

    fn set_variable(&mut self, variable: &VariableType, value: Vector3<f32>) {
        match variable {
            VariableType::AngularVelocity => self.angular_velocity = value,
            VariableType::Velocity => self.velocity = value,
            VariableType::AngularAcceleration => self.angular_acceleration = value,
            VariableType::Acceleration => self.acceleration = value,
            VariableType::Position => self.position = value,
            VariableType::Rotation => self.rotation = *UnitQuaternion::from_euler_angles(value.x, value.y, value.z).quaternion(),
        }
    }

    fn look_at(&mut self, at: Vector3<f32>) {
        self.rotation = *UnitQuaternion::face_towards(&(at - self.position), &Vector3::y()).quaternion();
    }
}
