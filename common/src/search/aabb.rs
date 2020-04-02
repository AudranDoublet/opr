use nalgebra::Vector3;
use std::f32;

#[derive(Clone, Copy, Debug)]
pub struct AABB {
    pub min: Vector3<f32>,
    pub max: Vector3<f32>,
}

impl AABB {
    pub fn new(min: Vector3<f32>, max: Vector3<f32>) -> AABB {
        AABB {
            min: min,
            max: max,
        }
    }

    pub fn empty() -> AABB {
        AABB {
            min: Vector3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            max: Vector3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
        }
    }

    pub fn new_from_pointset(mins: &[Vector3<f32>], maxs: &[Vector3<f32>]) -> AABB {
        let mut min = Vector3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
        let mut max = Vector3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);

        for p in mins {
            for i in 0..3 {
                min[i] = min[i].min(p[i]);
            }
        }

        for p in maxs {
            for i in 0..3 {
                max[i] = max[i].max(p[i]);
            }
        }

        AABB {
            min: min,
            max: max,
        }
    }

    pub fn is_inside(&self, p: Vector3<f32>) -> bool {
        self.min < p && p < self.max
    }

    pub fn intersects(&self, other: &AABB) -> bool {
        self.min <= other.max && self.max >= other.min
    }

    pub fn join(&self, other: &AABB) -> AABB {
        AABB::new_from_pointset(&[self.min, other.min], &[self.max, other.max])
    }

    pub fn grow(&self, p: Vector3<f32>) -> AABB {
        AABB::new_from_pointset(&[self.min, p], &[self.max, p])
    }

    pub fn center(&self) -> Vector3<f32> {
        (self.min + self.max) / 2.
    }

    pub fn surface(&self) -> f32 {
        let diff = self.max - self.min;
        let sq = diff.component_mul(&diff);

        2.0 * (sq.x + sq.y + sq.z)
    }

    pub fn largest_axis(&self) -> (usize, f32) {
        let sizes = self.max - self.min;

        let mut result = (0, sizes.x);

        if result.1 < sizes.y {
            result = (1, sizes.y);
        }

        if result.1 < sizes.z {
            result = (2, sizes.z);
        }

        result
    }
}
