use crate::search::{AABB, Bucket};
use std::f32;

use nalgebra::Vector3;

const EPSILON: f32 = 1e-5;

pub struct BVHParameters {
    pub max_depth: usize,
    pub max_shapes_in_leaf: usize,
    pub bucket_count: usize,
}

pub trait BVHShape {
    fn aabb(&self) -> AABB;
}

pub trait BVHShapeInside {
    fn is_inside(&self, aabb: &AABB) -> bool;
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
    fn build(params: &BVHParameters, depth: usize, points_indices: &Vec<usize>, points: &Vec<T>) -> BVHNode<T> {
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
            BVHNode::Leaf {
                shapes: points_indices.iter().map(|i| points[*i].clone()).collect()
            }
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


            BVHNode::Node {
                left_node: Box::new(BVHNode::build(params, depth + 1, &l, points)),
                right_node: Box::new(BVHNode::build(params, depth + 1, &r, points)),
                left_box: l_aabb,
                right_box: r_aabb,
            }
        }
    }
}

impl<T: BVHShape + BVHShapeInside + Clone> BVHNode<T> {
    pub fn search_shapes(&self, aabb: &AABB, f: &mut dyn Fn(&T)) {
        match self {
            BVHNode::Leaf { shapes } => {
                for s in shapes {
                    if s.is_inside(aabb) {
                        f(s);
                    }
                }
            },
            BVHNode::Node { left_node, right_node, left_box, right_box } => {
                if left_box.intersects(aabb) {
                    left_node.search_shapes(aabb, f);
                }

                if right_box.intersects(aabb) {
                    right_node.search_shapes(aabb, f);
                }
            },
        }
    }
}

#[derive(Debug)]
pub struct BVH<T: BVHShape + Clone> {
    root: BVHNode<T>,
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
        let root = BVHNode::build(params, 0, &(0..points.len()).collect(), points);

        BVH {
            root: root,
        }
    }
}

impl<T: BVHShape + Clone> Default for BVH<T> {
    fn default() -> BVH<T> {
        BVH::build(&Vec::new())
    }
}

impl<T: BVHShape + BVHShapeInside + Clone> BVH<T> {
    pub fn foreach_inside(&self, aabb: &AABB, f: &mut dyn Fn(&T)) {
        self.root.search_shapes(aabb, f);
    }
}

/** Implement BVH types for Vector3 */
impl BVHShape for Vector3<f32> {
    fn aabb(&self) -> AABB {
        AABB::new(*self, *self)
    }
}

impl BVHShapeInside for Vector3<f32> {
    fn is_inside(&self, aabb: &AABB) -> bool {
        aabb.is_inside(*self)
    }
}
