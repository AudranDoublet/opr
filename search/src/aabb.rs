use nalgebra::{Vector3, Matrix3};
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

    pub fn new_from_pointset(points: &[Vector3<f32>]) -> AABB {
        let mut min = Vector3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
        let mut max = Vector3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);

        for p in points {
            for i in 0..3 {
                min[i] = min[i].min(p[i]);
                max[i] = max[i].max(p[i]);
            }
        }

        AABB {
            min: min,
            max: max,
        }
    }

    pub fn new_from_pointset_mm(mins: &[Vector3<f32>], maxs: &[Vector3<f32>]) -> AABB {
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

    pub fn get(&self, i: usize) -> &Vector3<f32> {
        match i {
            0 => &self.min,
            _ => &self.max,
        }
    }

    pub fn is_inside(&self, p: Vector3<f32>) -> bool {
        self.min < p && p < self.max
    }

    pub fn intersects(&self, other: &AABB) -> bool {
        self.min <= other.max && self.max >= other.min
    }

    pub fn join(&self, other: &AABB) -> AABB {
        AABB::new_from_pointset_mm(&[self.min, other.min], &[self.max, other.max])
    }

    pub fn grow(&self, p: Vector3<f32>) -> AABB {
        AABB::new_from_pointset_mm(&[self.min, p], &[self.max, p])
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

    pub fn coord(&self, i: f32, j: f32, k: f32) -> Vector3<f32> {
        self.min + (self.max - self.min).component_mul(&Vector3::new(i, j, k))
    }

    pub fn transform(&self, rotation: &Matrix3<f32>, translation: &Vector3<f32>) -> AABB {
        let origin = rotation*self.min + translation;

        let dx = rotation*self.coord(1.0, 0.0, 0.0) + translation - origin;
        let dy = rotation*self.coord(0.0, 1.0, 0.0) + translation - origin;
        let dz = rotation*self.coord(0.0, 0.0, 1.0) + translation - origin;

        let mut min = Vector3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY);
        let mut max = Vector3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY);

        for x in 0..=1 {
            for y in 0..=1 {
                for z in 0..=1 {
                    let pos = origin + dx*x as f32 + dy*y as f32 + dz*z as f32;

                    for i in 0..3 {
                        min[i] = min[i].min(pos[i]);
                        max[i] = max[i].max(pos[i]);
                    }
                }
            }
        }

        AABB::new(min, max)
    }
}
