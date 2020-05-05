use search::{BVHShape, IntersectsBVHShape, Intersection, Ray, AABB};
use nalgebra::{Vector3, Vector2};

use crate::shapes::Shape;

#[derive(Clone, Copy)]
pub struct Plane
{
    pub axis: usize,
    pub position: f32,
    pub material: usize,
    pub id: usize,
}

impl Plane
{
    pub fn new(axis: usize, position: f32, material: usize, id: usize) -> Plane
    {
        Plane {
            axis,
            position,
            material,
            id,
        }
    }
}

impl IntersectsBVHShape for Plane {
    fn intersects(&self, ray: &Ray, self_hit_eps: f32) -> Option<Intersection>
    {
        // ray is parallel to plane
        if ray.direction[self.axis].abs() < 0.0001 {
            return None;
        }

        let t = (self.position - ray.origin[self.axis]) / ray.direction[self.axis];

        match t {
            t if t >= self_hit_eps => Some(Intersection {
                distance: t,
                u: 0.0,
                v: 0.0,
            }),
            _ => None,
        }
    }

    fn get_tex_coords(&self, _: f32, _: f32) -> Vector2<f32>
    {
        unimplemented!()
    }
}

impl BVHShape for Plane {
    fn aabb(&self) -> AABB {
        AABB::new_from_pointset(&[])
    }
}

impl Shape for Plane {
    fn clone_shape(&self) -> Box<dyn Shape + Sync + Send> {
        Box::new(*self)
    }

    fn material(&self) -> usize {
        self.material
    }

    fn shader(&self) -> Option<usize> {
        None
    }

    fn id(&self) -> usize {
        self.id
    }

    fn smoothed_normal(&self, r: &Ray, _: &Intersection) -> Vector3<f32> {
        let mut normal = Vector3::zeros();

        normal[self.axis] = 1.0;

        if normal.dot(&r.direction) > 0.0 {
            -normal
        } else {
            normal
        }
    }
}
