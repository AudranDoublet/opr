use serde_derive::*;

use nalgebra::Vector3;
use search::Ray;
use crate::vector3_from_const;

#[derive(Debug, Deserialize)]
pub struct Camera
{
    origin: Vector3<f32>,
    up: Vector3<f32>,
    front: Vector3<f32>,
    width: f32,
    height: f32,
}

impl Camera
{
    pub fn new_empty() -> Camera {
        Camera::new(vector3_from_const(0.), vector3_from_const(1.), vector3_from_const(1.), 512., 512.)
    }

    pub fn new(origin: Vector3<f32>, up: Vector3<f32>, front: Vector3<f32>, width: f32, height: f32) -> Camera
    {
        Camera {
            origin: origin,
            up: up,
            front: front,
            width: width,
            height: height,
        }
    }

    pub fn set_size(&mut self, width: f32, height: f32) {
        self.width = width;
        self.height = height;
    }

    fn aspect_ratio(&self) -> Vector3<f32> {
        let mut aspect_ratio = vector3_from_const(1.0);

        if self.width < self.height
        {
            aspect_ratio.y = self.height / self.width;
        }
        else
        {
            aspect_ratio.x = self.width / self.height;
        }

        aspect_ratio
    }

    fn right(&self) -> Vector3<f32> {
        self.up.cross(&self.front)
    }

    pub fn generate_ray(&self, mut x: f32, mut y: f32) -> Ray
    {
        let aspect_ratio = self.aspect_ratio();
        x = x * aspect_ratio.x;
        y = y * aspect_ratio.y;

        let h2 = self.height / 2.0;
        let w2 = self.width / 2.0;

        let dir = self.front + self.up * ((h2 - y) / h2) + self.right() * ((x - w2) / w2);

        Ray::new(self.origin, dir.normalize())
    }
}
