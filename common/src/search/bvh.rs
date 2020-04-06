use crate::search::{AABB, Bucket};
use std::f32;

use crate::mesh::Triangle;
use nalgebra::{Matrix3, Vector3};

const EPSILON: f32 = 1e-5;

pub struct BVHParameters {
    pub max_depth: usize,
    pub max_shapes_in_leaf: usize,
    pub bucket_count: usize,
}

pub trait BVHShape {
    fn aabb(&self) -> AABB;
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

    #[allow(dead_code)]
    fn dump_nodes(&self, id: usize, f: &mut dyn Fn(usize, &T)) {
        match self {
            BVHNode::Leaf{shapes} => {
                for v in shapes {
                    f(id, v);
                }
            },
            _ => ()
        }
    }

    #[allow(dead_code)]
    fn intersect_iter(&self,
            f: &mut dyn Fn(usize, &T),
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
                other.dump_nodes(1, f);
                self.dump_nodes(0, f);
            } else if let BVHNode::Node { left_box, right_box, left_node, right_node } = other {
                self.intersect_iter(f, rotation, translation, self_aabb, &modify(left_box), left_node);
                self.intersect_iter(f, rotation, translation, self_aabb, &modify(right_box), right_node);
            }
        } else if let BVHNode::Node { left_box: lb, right_box: rb, left_node: ln, right_node: rn } = self {
            if other.is_leaf() {
                ln.intersect_iter(f, rotation, translation, lb, other_aabb, other);
                rn.intersect_iter(f, rotation, translation, rb, other_aabb, other);
            } else if let BVHNode::Node { left_box, right_box, left_node, right_node } = other {
                let left_box = modify(left_box);
                let right_box = modify(right_box);

                ln.intersect_iter(f, rotation, translation, lb, &left_box, left_node);
                ln.intersect_iter(f, rotation, translation, lb, &right_box, right_node);
                rn.intersect_iter(f, rotation, translation, rb, &left_box, left_node);
                rn.intersect_iter(f, rotation, translation, rb, &right_box, right_node);
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

    #[allow(dead_code)]
    pub fn intersects(&self,
            other: &BVH<T>,
            f: &mut dyn Fn(usize, &T),
            rotation: &Matrix3<f32>,
            translation: &Vector3<f32>) {
        self.root.intersect_iter(f, rotation, translation,
                                 &self.aabb,
                                 &other.aabb.transform(rotation, translation), &other.root);
    }

    #[allow(dead_code)]
    pub fn root(&self) -> &Box<BVHNode<T>> {
        &self.root
    }
}

impl<T: BVHShape + Clone> Default for BVH<T> {
    fn default() -> BVH<T> {
        BVH::build(&Vec::new())
    }
}

/** Implement BVH types for Vector3 */
impl BVHShape for Triangle {
    fn aabb(&self) -> AABB {
        AABB::new_from_pointset(&[self.v1, self.v2, self.v3])
    }
}
