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

fn solve_quadratic(a: f32, b: f32, c: f32, eps: f32) -> Option<f32> {
    let discriminant = b * b - 4.0 * a * c;

    let x = if discriminant < 0.0 {
        -1.0
    } else if discriminant == 0.0 {
        -b / (2.0 * a)
    } else {
        let discriminant = discriminant.sqrt();
        // FIXME: we can simplify the formula
        let x1 = (-b - discriminant) / (2. * a);
        let x2 = (-b + discriminant) / (2. * a);
        // FIXME-END

        x1.max(x2)
    };

    if x < eps {
        None
    } else {
        Some(x)
    }
}

impl BVHShape for Sphere {
    fn aabb(&self) -> AABB {
        let radius = Vector3::new(self.radius, self.radius, self.radius);
        AABB::new(&self.center - radius, &self.center + radius)
    }
}

impl IntersectsBVHShape for Sphere {
    fn intersects(&self, ray: &Ray, self_hit_eps: f32) -> Option<Intersection>
    {
        let l = &ray.origin - &self.center;
        let (a, b, c) = (1., 2.0 * ray.direction.dot(&l), l.dot(&l) - self.radius.powi(2));

        if let Some(t) = solve_quadratic(a, b, c, self_hit_eps) {
            // FIXME: maybe we should add support UV
            Some(Intersection::new(t, 0., 0.))
            // FIXME-END
        } else {
            None
        }
    }

    fn get_tex_coords(&self, _: f32, _: f32) -> Vector2<f32>
    {
        unimplemented!()
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
        (i.position(ray)  - &self.center).normalize()
    }
}
