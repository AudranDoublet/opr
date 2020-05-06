use nalgebra::{Vector3, Quaternion, UnitQuaternion};

use std::sync::RwLock;

use crate::{Animation, VariableType, AnimationHandler};

pub struct Emitter {
    fluid_type: usize,
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

    emitting: bool,
    emit: bool,
    is_shot: bool,
}

impl Emitter {
    pub fn new(fluid_type: usize, position: Vector3<f32>, particle_velocity: f32,
                particle_radius: f32, particles: Vec<Vector3<f32>>, is_shot: bool) -> Emitter {
        Emitter {
            fluid_type: fluid_type,
            particle_velocity: particle_velocity,
            position: position,
            velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            rotation: Quaternion::identity(),
            acceleration: Vector3::zeros(),
            angular_acceleration: Vector3::zeros(),

            next_emit: RwLock::new(0.0),
            time_between_emits: particle_radius * 2. / particle_velocity,

            emitting: false,
            emit: false,
            particles: particles,
            is_shot: is_shot,
        }
    }

    pub fn emit(&self) -> Vec<(usize, Vector3<f32>, Vector3<f32>)> {
        if !self.emit {
            return Vec::new();
        }

        let rotation = UnitQuaternion::from_quaternion(self.rotation);
        let pvelocity = (rotation * Vector3::z()) * self.particle_velocity;

        let mut res = Vec::new();

        for particle in &self.particles {
            res.push((
                self.fluid_type,
                rotation * particle + self.position,
                pvelocity,
            ));
        }

        res
    }

    pub fn tick(&mut self, dt: f32, animation: &mut Animation) -> Vec<(usize, Vector3<f32>, Vector3<f32>)> {
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
            VariableType::LookAtRelative => unimplemented!(),
            VariableType::LookAt => unimplemented!(),
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
            VariableType::LookAtRelative => self.look_at(value + self.position),
            VariableType::LookAt => self.look_at(value),
        }
    }

    fn look_at(&mut self, at: Vector3<f32>) {
        let mut y = Vector3::y();
        let dir = (at - self.position).normalize();

        if dir.dot(&y).abs() > 0.99 {
            y = Vector3::x();
        }

        self.rotation = *UnitQuaternion::face_towards(&dir, &y).quaternion();
    }

    fn set_emit(&mut self, emit: bool) {
        self.emit = emit && (!self.is_shot || !self.emitting);
        self.emitting = emit;
    }
}
