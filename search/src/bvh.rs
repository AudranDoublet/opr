use crate::{AABB, Bucket, Ray, Intersection};
use std::f32;

use nalgebra::{Matrix3, Vector3, Vector2};

const EPSILON: f32 = 1e-5;

pub struct BVHParameters {
    pub max_depth: usize,
    pub max_shapes_in_leaf: usize,
    pub bucket_count: usize,
}

pub trait BVHShape {
    fn aabb(&self) -> AABB;
}

pub trait IntersectsBVHShape {
    fn intersects(&self, ray: &Ray) -> Option<Intersection>;

    fn get_tex_coords(&self, u: f32, v: f32) -> Vector2<f32>;
}

#[derive(Debug)]
pub enum BVHNode<T: BVHShape + Clone> {
    Leaf {
        shapes: Vec<T>,
    },
    Node {
        left_node: Box<BVHNode<T>>,
        right_node: Box<BVHNode<T>>,
        left_box: AABB,
        right_box: AABB,
    }
}

impl<T: BVHShape + Clone> BVHNode<T> {
    fn build(params: &BVHParameters, depth: usize, points_indices: &Vec<usize>, points: &Vec<T>) -> (AABB, Box<BVHNode<T>>) {
        let mut convex_hull = (AABB::empty(), AABB::empty());

        for &idx in points_indices {
            let s_aabb = points[idx].aabb();

            convex_hull = (
                convex_hull.0.join(&s_aabb),
                convex_hull.1.grow(s_aabb.center())
            )
        }

        let (largest_axis, largest_axis_length) = convex_hull.1.largest_axis();

        if depth >= params.max_depth
            || points_indices.len() <= params.max_shapes_in_leaf
            || largest_axis_length < EPSILON
        {
            (
                convex_hull.0,
                Box::new(BVHNode::Leaf {
                    shapes: points_indices.iter().map(|i| points[*i].clone()).collect()
                })
            )
        } else {
            let mut buckets = vec![Bucket::new(); params.bucket_count];
            let mut buckets_assignements = vec![Vec::new(); params.bucket_count];

            for &idx in points_indices {
                let aabb = points[idx].aabb();

                let relative = (aabb.center()[largest_axis] - convex_hull.1.min[largest_axis]) / largest_axis_length;
                let buck_num = (relative * (params.bucket_count as f32 - EPSILON)) as usize;

                buckets[buck_num].add_aabb(&aabb);
                buckets_assignements[buck_num].push(idx);
            }

            let mut min_cost = f32::INFINITY;
            let mut min_split = 0;
            let mut l_aabb = AABB::empty();
            let mut r_aabb = AABB::empty();

            for i in 0..params.bucket_count-1 {
                let (l, r) = buckets.split_at(i + 1);

                let bucket_l = Bucket::join_all(l);
                let bucket_r = Bucket::join_all(r);

                let cost = (bucket_l.cost() + bucket_r.cost()) / convex_hull.0.surface();

                if cost < min_cost {
                    min_cost = cost;
                    min_split = i;
                    l_aabb = bucket_l.aabb;
                    r_aabb = bucket_r.aabb;
                }
            }

            let (l, r) = buckets_assignements.split_at(min_split + 1);

            let l = l.concat();
            let r = r.concat();

            (
                convex_hull.0,
                Box::new(BVHNode::Node {
                    left_node: BVHNode::build(params, depth + 1, &l, points).1,
                    right_node: BVHNode::build(params, depth + 1, &r, points).1,
                    left_box: l_aabb,
                    right_box: r_aabb,
                })
            )
        }
    }

    #[allow(dead_code)]
    pub fn is_leaf(&self) -> bool {
        match self {
            BVHNode::Leaf {..} => true,
            _ => false,
        }
    }


    fn dump_nodes(&self, f: &mut Vec<(T, T)>, other: &BVHNode<T>) {
        match (self, other) {
            (BVHNode::Leaf{shapes: a}, BVHNode::Leaf{shapes: b}) => {
                for v in a {
                    for w in b {
                        f.push((v.clone(), w.clone()));
                    }
                }
            },
            _ => ()
        }
    }

    #[allow(dead_code)]
    fn intersect_iter(&self,
            vec: &mut Vec<(T, T)>,
            rotation: &Matrix3<f32>,
            translation: &Vector3<f32>,
            self_aabb: &AABB,
            other_aabb: &AABB, other: &BVHNode<T>) {

        if !self_aabb.intersects(other_aabb) {
            return;
        }

        let modify = |b: &AABB| b.transform(rotation, translation);

        if self.is_leaf() {
            if other.is_leaf() {
                self.dump_nodes(vec, other);
            } else if let BVHNode::Node { left_box, right_box, left_node, right_node } = other {
                self.intersect_iter(vec, rotation, translation, self_aabb, &modify(left_box), left_node);
                self.intersect_iter(vec, rotation, translation, self_aabb, &modify(right_box), right_node);
            }
        } else if let BVHNode::Node { left_box: lb, right_box: rb, left_node: ln, right_node: rn } = self {
            if other.is_leaf() {
                ln.intersect_iter(vec, rotation, translation, lb, other_aabb, other);
                rn.intersect_iter(vec, rotation, translation, rb, other_aabb, other);
            } else if let BVHNode::Node { left_box, right_box, left_node, right_node } = other {
                let left_box = modify(left_box);
                let right_box = modify(right_box);

                ln.intersect_iter(vec, rotation, translation, lb, &left_box, left_node);
                ln.intersect_iter(vec, rotation, translation, lb, &right_box, right_node);
                rn.intersect_iter(vec, rotation, translation, rb, &left_box, left_node);
                rn.intersect_iter(vec, rotation, translation, rb, &right_box, right_node);
            }
        }
    }

    fn intersect_one(&self, shape: &AABB, vec: &mut Vec<T>) {
        match self {
            BVHNode::Leaf { shapes } => {
                for v in shapes {
                    vec.push(v.clone())
                }
            },
            BVHNode::Node { left_box, right_box, left_node, right_node } => {
                if left_box.intersects(shape) {
                    left_node.intersect_one(shape, vec)
                }

                if right_box.intersects(shape) {
                    right_node.intersect_one(shape, vec)
                }
            }
        }
    }
}

impl<T: BVHShape + IntersectsBVHShape + Clone> BVHNode<T> {
    fn ray_intersects(&self, ray: &Ray, result: &mut (Intersection, Option<T>)) {
        match self {
            BVHNode::Leaf { shapes } => {
                for s in shapes {
                    if let Some(i) = s.intersects(ray) {
                        if i.distance < result.0.distance {
                            *result = (i, Some(s.clone()))
                        }
                    }
                }
            },
            BVHNode::Node { left_box, right_box, left_node, right_node } => {
                let a = ray.intersects_aabb(left_box).unwrap_or(std::f32::INFINITY);
                let b = ray.intersects_aabb(right_box).unwrap_or(std::f32::INFINITY);

                if a <= b && a < result.0.distance {
                    left_node.ray_intersects(ray, result);

                    if b < result.0.distance {
                        right_node.ray_intersects(ray, result);
                    }
                } else if b <= a && b < result.0.distance {
                    right_node.ray_intersects(ray, result);

                    if a < result.0.distance {
                        left_node.ray_intersects(ray, result);
                    }
                }
            }
        }
    }
}

#[derive(Debug)]
pub struct BVH<T: BVHShape + Clone> {
    root: Box<BVHNode<T>>,
    aabb: AABB,
}

impl<T: BVHShape + Clone> BVH<T> {
    pub fn build(points: &Vec<T>) -> BVH<T> {
        BVH::build_params(
            &BVHParameters {
                max_depth: 50,
                max_shapes_in_leaf: 1,
                bucket_count: 6,
            }, points
        )
    }

    pub fn build_params(params: &BVHParameters, points: &Vec<T>) -> BVH<T> {
        let (aabb, root) = BVHNode::build(params, 0, &(0..points.len()).collect(), points);

        BVH {
            root: root,
            aabb: aabb,
        }
    }

    pub fn aabb(&self) -> AABB {
        self.aabb
    }

    pub fn intersects(&self,
            other: &BVH<T>,
            rotation: &Matrix3<f32>,
            translation: &Vector3<f32>) -> Vec<(T, T)> {
        let mut result = Vec::new();

        self.root.intersect_iter(&mut result, rotation, translation,
                                 &self.aabb,
                                 &other.aabb.transform(rotation, translation), &other.root);

        result
    }

    pub fn intersect_one(&self, shape: &AABB) -> Vec<T> {
        let mut result = Vec::new();

        if self.aabb().intersects(shape) {
            self.root.intersect_one(shape, &mut result)
        }

        result
    }
}

impl<T: BVHShape + IntersectsBVHShape + Clone> BVH<T> {
    pub fn ray_intersect(&self, ray: &Ray) -> Option<(Intersection, T)> {
        let mut result = (Intersection::new_empty(), None);

        if !ray.intersects_aabb(&self.aabb()).is_none() {
            self.root.ray_intersects(ray, &mut result);
        }

        if let Some(s) = result.1 {
            Some((result.0, s))
        } else {
            None
        }
    }
}

impl<T: BVHShape + Clone> Default for BVH<T> {
    fn default() -> BVH<T> {
        BVH::build(&Vec::new())
    }
}

#[derive(Debug, Clone)]
pub struct Sphere {
    pub position: Vector3<f32>,
    pub radius: f32,
}

impl Sphere {
    pub fn new(position: &Vector3<f32>, radius: f32) -> Sphere {
        Sphere {
            position: *position,
            radius: radius,
        }
    }
}

impl BVHShape for Sphere {
    fn aabb(&self) -> AABB {
        let radius = Vector3::new(self.radius, self.radius, self.radius);

        AABB::new(self.position - radius, self.position + radius)
    }
}
