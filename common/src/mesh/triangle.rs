use nalgebra::Vector3;

#[derive(Clone, Copy)]
pub struct Triangle
{
    pub v1: Vector3<f32>,
    pub v2: Vector3<f32>,
    pub v3: Vector3<f32>,
    pub v1_normal: Vector3<f32>,
    pub v2_normal: Vector3<f32>,
    pub v3_normal: Vector3<f32>,
}

impl Triangle
{
    pub fn new_no_normals(v1: Vector3<f32>, v2: Vector3<f32>, v3: Vector3<f32>) -> Triangle
    {
        //let normal = (v3 - v1).cross(&(v2 - v1));
        let normal = (v2 - v1).cross(&(v3 - v1));

        Triangle {
            v1: v1,
            v2: v2,
            v3: v3,
            v1_normal: normal,
            v2_normal: normal,
            v3_normal: normal,
        }
    }

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
               v1_normal: Vector3<f32>, v2_normal: Vector3<f32>, v3_normal: Vector3<f32>) -> Triangle
    {
        Triangle {
            v1: v1,
            v2: v2,
            v3: v3,
            v1_normal: v1_normal,
            v2_normal: v2_normal,
            v3_normal: v3_normal,
        }
    }

    pub fn smoothed_normal(&self, s: f32, t: f32) -> Vector3<f32> {
        self.v1_normal * (1. - s - t)
            + self.v2_normal * s
            + self.v3_normal * t
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
        let sign = match self.smoothed_normal(s, t).dot(&diff) {
            v if v >= 0. => 1.0,
            _ => -1.0,
        };

        (distance, sign)
    }
}
