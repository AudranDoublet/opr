use nalgebra::Vector3;
use crate::AABB;

pub struct Ray {
    pub origin: Vector3<f32>,
    pub direction: Vector3<f32>,

    inv_direction: Vector3<f32>,
    sign: Vector3<usize>,
}

pub struct Intersection {
    pub distance: f32,
    pub u: f32,
    pub v: f32,
}

impl Intersection {
    pub fn new(distance: f32, u: f32, v: f32) -> Intersection {
        Intersection {
            distance: distance,
            u: u,
            v: v,
        }
    }

    pub fn new_empty() -> Intersection {
        Intersection::new(std::f32::INFINITY, 0.0, 0.0)
    }

    pub fn position(&self, ray: &Ray) -> Vector3<f32> {
        ray.origin + self.distance * ray.direction
    }
}

impl Ray {
    pub fn new(origin: Vector3<f32>, direction: Vector3<f32>) -> Ray {
        let direction = direction.normalize();
        Ray {
            origin: origin,
            direction: direction,
            inv_direction: Vector3::new(1.0 / direction.x, 1.0 / direction.y, 1.0 / direction.z),
            sign: Vector3::new(
                (direction.x < 0.0) as usize,
                (direction.y < 0.0) as usize,
                (direction.z < 0.0) as usize,
            ),
        }
    }

    pub fn intersects_aabb(&self, aabb: &AABB) -> Option<f32> {
        let origin = &self.origin;
        let sign = &self.sign;

        let mut ray_min = (aabb.get(sign.x).x - origin.x) * self.inv_direction.x;
        let mut ray_max = (aabb.get(1 - sign.x).x - origin.x) * self.inv_direction.x;

        let y_min = (aabb.get(sign.y).y - origin.y) * self.inv_direction.y;
        let y_max = (aabb.get(1 - sign.y).y - origin.y) * self.inv_direction.y;

        if (ray_min > y_max) || (y_min > ray_max) {
            return None;
        }

        if y_min > ray_min {
            ray_min = y_min;
        }

        if y_max < ray_max {
            ray_max = y_max;
        }

        let z_min = (aabb.get(sign.z).z - origin.z) * self.inv_direction.z;
        let z_max = (aabb.get(1 - sign.z).z - origin.z) * self.inv_direction.z;

        if (ray_min > z_max) || (z_min > ray_max) {
            return None;
        }

        if z_min > ray_min {
            ray_min = z_min;
        }

        if z_max < ray_max {
            ray_max = z_max;
        }

        if ray_min < 0.0 {
            if ray_max > 0.0 {
                Some(0.0)
            } else {
                None
            }
        } else {
            Some(ray_min)
        }
    }
}
