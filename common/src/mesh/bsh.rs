use crate::mesh::Triangle;
use nalgebra::Vector3;

#[derive(Copy, Clone)]
struct RecResult
{
    distance: f32,
    sign: f32,
}

pub enum BoundingSphereHierarchy
{
    Leaf {
        triangles: Vec<Triangle>,
        center: Vector3<f32>,
        radius: f32,
    },
    Node {
        r_split: f32,
        l_split: f32,
        center: Vector3<f32>,
        radius: f32,
        r_child: Box<BoundingSphereHierarchy>,
        l_child: Box<BoundingSphereHierarchy>,
    }
}

impl BoundingSphereHierarchy
{
    fn max_dist(triangles: &[Triangle], coord: usize) -> (Vector3<f32>, f32) {
        let mut min = std::f32::INFINITY;
        let mut max = std::f32::NEG_INFINITY;

        let mut emin = Vector3::zeros();
        let mut emax = Vector3::zeros();

        for i in 0..triangles.len() {
            for j in 0..3 {
                let v = *triangles[i].vertex(j).index(coord);

                if v < min {
                    min = v;
                    emin = triangles[i].vertex(j);
                }

                if v > max {
                    max = v;
                    emax = triangles[i].vertex(j);
                }
            }
        }

        ((emax + emin) / 2., max - min)
    }

    fn compute_sphere(triangles: &[Triangle]) -> (Vector3<f32>, f32) {
        let (cx, mx) = BoundingSphereHierarchy::max_dist(triangles, 0);
        let (cy, my) = BoundingSphereHierarchy::max_dist(triangles, 1);
        let (cz, mz) = BoundingSphereHierarchy::max_dist(triangles, 2);

        let mut max = mx;
        let mut center = cx;

        if my > max {
            max = my;
            center = cy;
        }

        if mz > max {
            max = mz;
            center = cz;
        }

        for i in 0..triangles.len() {
            for j in 0..3 {
                let dist = triangles[i].vertex(j).norm_squared();

                // if the point isn't in the circle, extend it
                if dist > max {
                    max = dist;
                }
            }
        }

        (center, max)
    }

    fn new_rec(triangles: &mut [Triangle], depth: usize, max_depth: usize) -> (BoundingSphereHierarchy, f32, f32) {
        let (center, sq_radius) = BoundingSphereHierarchy::compute_sphere(triangles);

        let node = if triangles.len() < 10 || depth > max_depth {
            BoundingSphereHierarchy::Leaf {
                center: center,
                radius: sq_radius.sqrt(),
                triangles: triangles.to_vec()
            }
        } else {
            triangles.sort_unstable_by(
                |a, b| a.mean(depth).partial_cmp(&b.mean(depth)).unwrap()
            );

            let med = triangles.len() / 2;

            let (l_child, _, max) = BoundingSphereHierarchy::new_rec(&mut triangles[..=med], depth + 1, max_depth);
            let (r_child, min, _) = BoundingSphereHierarchy::new_rec(&mut triangles[med+1..], depth + 1, max_depth);

            BoundingSphereHierarchy::Node {
                l_split: max,
                r_split: min,
                l_child: Box::new(l_child),
                r_child: Box::new(r_child),
                center: center,
                radius: sq_radius.sqrt(),
            }
        };

        let mut min = core::f32::INFINITY;
        let mut max = core::f32::NEG_INFINITY;

        if depth > 0 {
            for t in triangles {
                min = min.min(t.min(depth - 1));
                max = max.max(t.max(depth - 1));
            }
        }

        (node, min, max)
    }

    pub fn new(triangles: &mut [Triangle], max_depth: usize) -> BoundingSphereHierarchy {
        BoundingSphereHierarchy::new_rec(triangles, 0, max_depth).0
    }

    fn minimal_distance_sphere(&self, p: Vector3<f32>) -> f32 {
        match self {
            BoundingSphereHierarchy::Leaf{center, radius, ..} => (p - center).norm() - radius,
            BoundingSphereHierarchy::Node{center, radius, ..} => (p - center).norm() - radius,
        }
    }

    fn minimal_distance_left(&self, coord: usize, p: Vector3<f32>) -> f32 {
        match self {
            BoundingSphereHierarchy::Leaf{..} => self.minimal_distance_sphere(p).max(0.0),
            BoundingSphereHierarchy::Node{l_split, l_child, ..} => (p[coord % 3] - l_split).max(l_child.minimal_distance_sphere(p)).max(0.0)
        }
    }

    fn minimal_distance_right(&self, coord: usize, p: Vector3<f32>) -> f32 {
        match self {
            BoundingSphereHierarchy::Leaf{..} => self.minimal_distance_sphere(p).max(0.0),
            BoundingSphereHierarchy::Node{r_split, r_child, ..} => (r_split - p[coord % 3]).min(r_child.minimal_distance_sphere(p)).max(0.0)
        }
    }

    fn minimal_signed_distance_rec(&self, depth: usize, p: Vector3<f32>, mut result: RecResult) -> RecResult {
        match self {
            BoundingSphereHierarchy::Leaf{triangles, ..} => {
                for triangle in triangles {
                    let (dist, sign) = triangle.signed_distance(p);

                    if dist <= result.distance {
                        result = RecResult {
                            distance: dist,
                            sign: sign,
                        };
                    }
                }

                result
            },
            BoundingSphereHierarchy::Node{r_child, l_child, ..} => {
                let rdist = self.minimal_distance_right(depth, p).powi(2);
                let ldist = self.minimal_distance_left(depth, p).powi(2);

                if ldist <= rdist && ldist <= result.distance {
                    result = l_child.minimal_signed_distance_rec(depth + 1, p, result);

                    if rdist <= result.distance {
                        result = r_child.minimal_signed_distance_rec(depth + 1, p, result);
                    }
                } else if rdist <= result.distance {
                    result = r_child.minimal_signed_distance_rec(depth + 1, p, result);

                    if ldist <= result.distance {
                        result = l_child.minimal_signed_distance_rec(depth + 1, p, result);
                    }
                }

                result
            }
        }
    }

    pub fn minimal_signed_distance(&self, p: Vector3<f32>) -> f32 {
        let result = self.minimal_signed_distance_rec(0, p, RecResult {
            distance: std::f32::INFINITY,
            sign: 1.0,
        });

        result.distance.sqrt() * result.sign
    }
}
