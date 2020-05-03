use nalgebra::{Vector2, Vector3};
use search::{AABB, BVHShape, Intersection, IntersectsBVHShape, Ray};

use crate::shapes::Shape;

const EPSILON: f32 = 0.0000001;

#[derive(Clone, Copy)]
pub struct Triangle
{
    pub v1: Vector3<f32>,
    pub v2: Vector3<f32>,
    pub v3: Vector3<f32>,
    pub v1_normal: Vector3<f32>,
    pub v2_normal: Vector3<f32>,
    pub v3_normal: Vector3<f32>,
    pub tex1: Vector2<f32>,
    pub tex2: Vector2<f32>,
    pub tex3: Vector2<f32>,
    pub material: usize,
    pub id: usize,
}

impl Triangle
{
    pub fn new(v1: Vector3<f32>, v2: Vector3<f32>, v3: Vector3<f32>,
               v1_normal: Vector3<f32>, v2_normal: Vector3<f32>, v3_normal: Vector3<f32>,
               tex1: Vector2<f32>, tex2: Vector2<f32>, tex3: Vector2<f32>,
               material: usize, id: usize) -> Triangle
    {
        Triangle {
            v1: v1,
            v2: v2,
            v3: v3,
            v1_normal: v1_normal,
            v2_normal: v2_normal,
            v3_normal: v3_normal,
            tex1: tex1,
            tex2: tex2,
            tex3: tex3,
            material: material,
            id: id,
        }
    }

    pub fn mean(&self, coord: usize) -> f32 {
        (self.v1[coord] + self.v2[coord] + self.v3[coord]) / 3.0
    }

    pub fn max(&self, coord: usize) -> f32 {
        let mut max = self.v1[coord];

        if max < self.v2[coord] {
            max = self.v2[coord];
        }

        if max > self.v3[coord] { max } else { self.v3[coord] }
    }

    pub fn min(&self, coord: usize) -> f32 {
        let mut min = self.v1[coord];

        if min > self.v2[coord] {
            min = self.v2[coord];
        }

        if min < self.v3[coord] { min } else { self.v3[coord] }
    }

    pub fn normal(&self) -> Vector3<f32> {
        let ab = &self.v2 - &self.v1;
        let ac = &self.v3 - &self.v1;

        return ab.cross(&ac);
    }
}

impl BVHShape for Triangle {
    fn aabb(&self) -> AABB {
        AABB::new_from_pointset(&[self.v1, self.v2, self.v3])
    }
}

impl IntersectsBVHShape for Triangle {
    fn intersects(&self, r: &Ray, self_hit_eps: f32) -> Option<Intersection>
    {
        let edge1 = &self.v2 - &self.v1;
        let edge2 = &self.v3 - &self.v1;

        let h = r.direction.cross(&edge2);
        let a = edge1.dot(&h);

        if a > -EPSILON && a < EPSILON // ray is parallel to the triangle
        {
            return None;
        }

        let f = 1.0 / a;
        let s = &r.origin - &self.v1;
        let u = f * s.dot(&h);

        if u < 0.0 || u > 1.0
        {
            return None;
        }

        let q = s.cross(&edge1);
        let v = f * r.direction.dot(&q);

        if v < 0.0 || u + v > 1.0
        {
            return None;
        }

        match f * edge2.dot(&q)
        {
            t if t >= self_hit_eps => Some(Intersection::new(t, u, v)),
            _ => None
        }
    }

    fn get_tex_coords(&self, u: f32, v: f32) -> Vector2<f32>
    {
        &self.tex1 + &self.tex2 * u + &self.tex3 * v
    }
}

impl Shape for Triangle {
    fn clone_shape(&self) -> Box<dyn Shape + Sync + Send> {
        Box::new(*self)
    }

    fn material(&self) -> usize {
        self.material
    }

    fn id(&self) -> usize {
        self.id
    }

    fn smoothed_normal(&self, _: &Ray, i: &Intersection) -> Vector3<f32> {
        (&self.v1_normal * (1.0 - i.u - i.v)
            + &self.v2_normal * i.u
            + &self.v3_normal * i.v).normalize()
    }
}
