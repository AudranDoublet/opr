use nalgebra::{Vector3, Vector2};
use search::{BVHShape, IntersectsBVHShape, Intersection, Ray, AABB};

#[derive(Clone, Copy, Debug)]
pub struct Triangle
{
    pub v1: Vector3<f32>,
    pub v2: Vector3<f32>,
    pub v3: Vector3<f32>,
    pub v1_normal: Vector3<f32>,
    pub v2_normal: Vector3<f32>,
    pub v3_normal: Vector3<f32>,
    pub e1_normal: Vector3<f32>,
    pub e2_normal: Vector3<f32>,
    pub e3_normal: Vector3<f32>,
}

impl Triangle
{
    pub fn mean(&self, coord: usize) -> f32 {
        let coord = coord % 3;
        (*self.v1.index(coord) + *self.v2.index(coord) + *self.v3.index(coord)) / 3.0
    }

    pub fn max(&self, coord: usize) -> f32 {
        let coord = coord % 3;
        let mut max = *self.v1.index(coord);

        if max < *self.v2.index(coord) {
            max = *self.v2.index(coord);
        }

        if max > *self.v3.index(coord) { max } else { *self.v3.index(coord) }
    }

    pub fn min(&self, coord: usize) -> f32 {
        let coord = coord % 3;
        let mut min = *self.v1.index(coord);

        if min > *self.v2.index(coord) {
            min = *self.v2.index(coord);
        }

        if min < *self.v3.index(coord) { min } else { *self.v3.index(coord) }
    }

    pub fn new(v1: Vector3<f32>, v2: Vector3<f32>, v3: Vector3<f32>,
               v1_normal: Vector3<f32>, v2_normal: Vector3<f32>, v3_normal: Vector3<f32>,
               e1_normal: Vector3<f32>, e2_normal: Vector3<f32>, e3_normal: Vector3<f32>) -> Triangle
    {
        Triangle {
            v1: v1,
            v2: v2,
            v3: v3,
            v1_normal: v1_normal,
            v2_normal: v2_normal,
            v3_normal: v3_normal,
            e1_normal: e1_normal,
            e2_normal: e2_normal,
            e3_normal: e3_normal,
        }
    }

    pub fn smoothed_normal(&self, s: f32, t: f32) -> Vector3<f32> {
        self.v1_normal * (1. - s - t)
            + self.v2_normal * s
            + self.v3_normal * t
    }

    pub fn surface_normal(&self) -> Vector3<f32> {
        self.edge(0).cross(&self.edge(1))
    }

    pub fn vertex(&self, i: usize) -> Vector3<f32> {
        match i {
            0 => self.v1,
            1 => self.v2,
            2 => self.v3,
            _ => panic!("unknown vertex")
        }
    }

    pub fn edge(&self, i: usize) -> Vector3<f32> {
        match i {
            0 => self.v2 - self.v1,
            1 => self.v3 - self.v1,
            2 => self.v3 - self.v2,
            _ => panic!("unknown edge")
        }
    }

    /**
     * Returns barycentric coordinates of triangle point that is the nearest from p
     * Source: https://www.geometrictools.com/Documentation/DistancePoint3Triangle3.pdf
     */
    pub fn nearest_point(&self, p: Vector3<f32>) -> (usize, f32, f32) {
        let diff = self.v1 - p;

        let a = self.edge(0).norm_squared();
        let b = self.edge(0).dot(&self.edge(1));
        let c = self.edge(1).dot(&self.edge(1));
        let d = self.edge(0).dot(&diff);
        let e = self.edge(1).dot(&diff);

        let s = b*e - c*d;
        let t = b*d - a*e;
        let det = a*c - b*b;

        if s + t <= det
        {
            if s < 0. && t < 0. // region 4
            {
                if d < 0. {
                    (4, (-d/a).clamp(0.0, 1.0), 0.0)
                } else {
                    (4, 0.0, (-e/c).clamp(0.0, 1.0))
                }
            }
            else if s < 0. // region 3
            {
                (3, 0.0, (-e/c).clamp(0.0, 1.0))
            }
            else if t < 0. // region 5
            {
                (5, 0.0, (-d/a).clamp(0.0, 1.0))
            }
            else // region 0
            {
                (0, s / det, t / det)
            }
        }
        else
        {
            if s < 0. // region 2
            {
                let v1 = b + d;
                let v2 = c + e;

                if v2 > v1 {
                    let s = ((v2 - v1) / (a - 2.0*b + c)).clamp(0.0, 1.0);
                    (2, s, 1. - s)
                } else {
                    (2, 0., (-e/c).clamp(0.0, 1.0))
                }
            }
            else if t < 0. // region 6
            {
                let v1 = b + e;
                let v2 = a + d;

                if v2 > v1 {
                    let t = ((v2 - v1) / (a - 2.0*b + c)).clamp(0.0, 1.0);
                    (6, 1. - t, t)
                } else {
                    (6, 0., (-d/a).clamp(0.0, 1.0))
                }
            }
            else // region 1
            {
                let s = ((c + e - b - d) / (a - 2.0*b + c)).clamp(0.0, 1.0);
                (1, s, 1. - s)
            }
        }
    }

    pub fn normal(&self, s: f32, t: f32) -> Vector3<f32> {
        let eq = |a: f32, b: f32| (a - b).abs() < 1e-5;

        assert!(s + t < 1. + 1e-5);

        if eq(s, 0.0) {
            if eq(t, 0.0) {
                self.v1_normal
            } else if eq(t, 1.0) {
                self.v3_normal
            } else {
                self.e2_normal
            }
        } else if eq(s, 1.0) {
            self.v2_normal
        } else if eq(t, 0.0) {
            self.e1_normal
        } else if eq(s + t, 1.0) {
            self.e3_normal
        } else {
            self.v1_normal + self.v2_normal + self.v3_normal
        }
    }

    /**
     * Compute signed minimal distance between a point and a triangle
     *
     * \return unsigned distance
     * \return distance sign (-1 or 1)
     */
    pub fn signed_distance(&self, p: Vector3<f32>) -> (f32, f32) {
        let (_region, s, t) = self.nearest_point(p);

        let nearest = self.edge(0) * s + self.edge(1) * t + self.v1;

        let diff = p - nearest;

        let distance = diff.norm_squared();

        let normal = self.normal(s, t).normalize();

        let sign = match normal.dot(&diff.normalize()) {
            v if v >= 0. => 1.0,
            _ => -1.0,
        };

        (distance, sign)
    }

    /**
     * Part of Mesh mass-properties algorithm
     * Source: https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
     */
    pub fn triangle_mass_properties(&self, intg: &mut [f32; 10]) {
        let subexpression = |w0, w1, w2| {
            let t0 = w0 + w1;
            let t1 = w0 * w0;
            let t2 = t1 + w1*t0;

            let f1 = t0 + w2;
            let f2 = t2 + w2*f1;
            let f3 = w0*t1 + w1*t2 + w2*f2;

            let g0 = f2 + w0*(f1 + w0);
            let g1 = f2 + w1*(f1 + w1);
            let g2 = f2 + w2*(f1 + w2);

            (Vector3::new(f1, f2, f3), Vector3::new(g0, g1, g2))
        };

        let d = self.surface_normal();

        let (fx, gx) = subexpression(self.v1.x, self.v2.x, self.v3.x);
        let (fy, gy) = subexpression(self.v1.y, self.v2.y, self.v3.y);
        let (fz, gz) = subexpression(self.v1.z, self.v2.z, self.v3.z);

        intg[0] += d.x * fx.x;

        intg[1] += d.x * fx.y;
        intg[2] += d.y * fy.y;
        intg[3] += d.z * fz.y;

        intg[4] += d.x * fx.z;
        intg[5] += d.y * fy.z;
        intg[6] += d.z * fz.z;

        intg[7] += d.x * (self.v1.y*gx.x + self.v2.y*gx.y + self.v3.y*gx.z);
        intg[8] += d.y * (self.v1.z*gy.x + self.v2.z*gy.y + self.v3.z*gy.z);
        intg[9] += d.z * (self.v1.x*gz.x + self.v2.x*gz.z + self.v3.x*gz.z);
    }
}

const EPSILON: f32 = 0.0000001;
const SELF_HIT_EPSILON: f32 = 0.00;

impl BVHShape for Triangle {
    fn aabb(&self) -> AABB {
        AABB::new_from_pointset(&[self.v1, self.v2, self.v3])
    }
}

impl IntersectsBVHShape for Triangle {
    fn intersects(&self, r: &Ray) -> Option<Intersection>
    {
        let edge1 = self.v2 - self.v1;
        let edge2 = self.v3 - self.v1;

        let h = r.direction.cross(&edge2);
        let a = edge1.dot(&h);

        if a > -EPSILON && a < EPSILON // ray is parallel to the triangle
        {
            return None;
        }

        let f = 1.0 / a;
        let s = r.origin - self.v1;
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
            t if t >= SELF_HIT_EPSILON => Some(Intersection::new(t, u, v)),
            _ => None
        }
    }

    fn get_tex_coords(&self, _: f32, _: f32) -> Vector2<f32>
    {
        Vector2::new(0.0, 0.0)
    }
}
