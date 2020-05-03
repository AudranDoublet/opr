use search::{BVHShape, IntersectsBVHShape, Intersection, Ray, AABB};
use nalgebra::{Vector3, Vector2};

use crate::shapes::Shape;

#[derive(Clone, Copy)]
pub struct Sphere
{
    pub center: Vector3<f32>,
    pub radius: f32,
    pub material: usize,
    pub id: usize,
}

impl Sphere
{
    pub fn new(center: Vector3<f32>, radius: f32, material: usize, id: usize) -> Sphere
    {
        assert!(radius > 0.0);

        Sphere {
            center,
            radius,
            material,
            id,
        }
    }
}

impl BVHShape for Sphere {
    fn aabb(&self) -> AABB {
        let radius = Vector3::new(self.radius, self.radius, self.radius);

        AABB::new(self.center - radius, self.center + radius)
    }
}

impl IntersectsBVHShape for Sphere {
    fn intersects(&self, r: &Ray, self_hit_eps: f32) -> Option<Intersection>
    {
        // FIXME
        None
    }

    fn get_tex_coords(&self, _: f32, _: f32) -> Vector2<f32>
    {
        Vector2::zeros()
    }
}

impl Shape for Sphere {
    fn clone_shape(&self) -> Box<dyn Shape + Sync + Send> {
        Box::new(*self)
    }

    fn material(&self) -> usize {
        self.material
    }

    fn id(&self) -> usize {
        self.id
    }

    fn smoothed_normal(&self, ray: &Ray, i: &Intersection) -> Vector3<f32> {
        let intersection = ray.origin + ray.direction * i.distance;

        (intersection - self.center).normalize()
    }
}
