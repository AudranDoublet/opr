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

pub struct IntersectionGroup<T: Clone> {
    pub intersections: Vec<(Intersection, T)>,

    pub min_distance: f32,
    pub epsilon: f32,
}

impl<T: Clone> IntersectionGroup<T> {
    pub fn new(epsilon: f32) -> IntersectionGroup<T> {
        IntersectionGroup {
            intersections: Vec::new(),
            min_distance: std::f32::INFINITY,
            epsilon: epsilon,
        }
    }

    #[inline]
    pub fn add_intersection(&mut self, shape: &T, intersection: Intersection) {
        let epsilon = self.epsilon;

        if intersection.distance < self.min_distance {
            self.min_distance = intersection.distance;

            self.intersections.retain(|(i, _)| i.distance - intersection.distance < epsilon);
            self.intersections.push((intersection, shape.clone()));
        } else if intersection.distance - self.min_distance < epsilon {
            self.intersections.push((intersection, shape.clone()));
        }
    }

    #[inline]
    pub fn end(&mut self) {
        self.intersections.sort_unstable_by(|(a, _), (b, _)| a.distance.partial_cmp(&b.distance).unwrap());
    }

    #[inline]
    pub fn get(&self, i: usize) -> Option<&(Intersection, T)> {
        self.intersections.get(i)
    }

    #[inline]
    pub fn max_allowed_distance(&self) -> f32 {
        self.min_distance + self.epsilon
    }
}

impl Ray {
    pub fn new(origin: Vector3<f32>, direction: Vector3<f32>) -> Ray {
        let classify = |v: f32| match v {
            v if v.abs() < 1e-4 => 2,
            v if v < 0.0 => 1,
            _ => 0,
        };

        let direction = direction.normalize();
        Ray {
            origin: origin,
            direction: direction,
            inv_direction: Vector3::new(1.0 / direction.x, 1.0 / direction.y, 1.0 / direction.z),
            sign: Vector3::new(
                classify(direction.x),
                classify(direction.y),
                classify(direction.z),
            ),
        }
    }

    pub fn intersects_aabb(&self, aabb: &AABB) -> Option<f32> {
        let origin = &self.origin;
        let sign = &self.sign;

        let mut ray_min = std::f32::NEG_INFINITY;
        let mut ray_max = std::f32::INFINITY;

        if sign.x != 2 {
            ray_min = (aabb.get(sign.x).x - origin.x) * self.inv_direction.x;
            ray_max = (aabb.get(1 - sign.x).x - origin.x) * self.inv_direction.x;
        }

        if sign.y != 2 {
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
        }

        if sign.z != 2 {
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
