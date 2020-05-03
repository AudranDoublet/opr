use std::ops::Deref;

use nalgebra::{Vector2, Vector3};
use search::{BVHShape, IntersectsBVHShape, Intersection, Ray, AABB};
pub use sphere::*;
pub use triangle::*;

mod triangle;
mod sphere;

pub trait Shape: BVHShape + IntersectsBVHShape {
    fn clone_shape(&self) -> Box<dyn Shape + Sync + Send>;

    fn material(&self) -> usize;

    fn id(&self) -> usize;

    fn smoothed_normal(&self, ray: &Ray, intersection: &Intersection) -> Vector3<f32>;
}

impl BVHShape for Box<dyn Shape + Sync + Send> {
    fn aabb(&self) -> AABB {
        self.deref().aabb()
    }
}

impl IntersectsBVHShape for Box<dyn Shape + Sync + Send> {
    fn intersects(&self, r: &Ray, self_hit_eps: f32) -> Option<Intersection> {
        self.deref().intersects(r, self_hit_eps)
    }

    fn get_tex_coords(&self, u: f32, v: f32) -> Vector2<f32> {
        self.deref().get_tex_coords(u, v)
    }
}

impl Clone for Box<dyn Shape + Sync + Send> {
    fn clone(&self) -> Box<dyn Shape + Sync + Send> {
        self.clone_shape()
    }
}
