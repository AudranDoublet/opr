use crate::{Animation, VariableType, AnimationHandler};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct Camera
{
    position: Vector3<f32>,
    velocity: Vector3<f32>,
    angular_velocity: Vector3<f32>,
    rotation: Quaternion<f32>,

    acceleration: Vector3<f32>,
    angular_acceleration: Vector3<f32>,
}

impl Camera {
    pub fn new(position: Vector3<f32>) -> Camera {
        Camera {
            position: position,
            velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            rotation: Quaternion::identity(),
            acceleration: Vector3::zeros(),
            angular_acceleration: Vector3::zeros(),
        }
    }

    pub fn position(&self) -> Vector3<f32> {
        self.position
    }

    pub fn up(&self) -> Vector3<f32> {
        UnitQuaternion::from_quaternion(self.rotation) * Vector3::y()
    }

    pub fn forward(&self) -> Vector3<f32> {
        UnitQuaternion::from_quaternion(self.rotation) * Vector3::z()
    }

    pub fn tick(&mut self, dt: f32, animation: &mut Animation) {
        animation.animate(self, dt, false);

        self.velocity += self.acceleration * dt;
        self.position += self.velocity * dt;

        self.angular_velocity += self.angular_acceleration * dt;

        let q = Quaternion::new(0.0, self.angular_velocity.x, self.angular_velocity.y, self.angular_velocity.z);

        self.rotation += (dt / 2.) * (q * self.rotation);
        self.rotation.normalize_mut();
    }
}

impl AnimationHandler for Camera {
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
        let mut y = Vector3::y();
        let dir = at - self.position;

        if dir.normalize().dot(&y).abs() > 0.99 {
            y = Vector3::x();
        }

        self.rotation = *UnitQuaternion::face_towards(&dir, &y).quaternion();
    }

    fn set_emit(&mut self, _: bool) { }
}
