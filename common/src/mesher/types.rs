use std::f32;

use nalgebra::Vector3;
use crate::kernels::Kernel;

pub type VertexWorld = Vector3<f32>;
pub type VertexLocal = Vector3<i32>;

pub type EdgeLocalExtremes = (VertexLocal, VertexLocal);
pub type EdgeIndices = (usize, usize);

pub type Triangle = (VertexWorld, VertexWorld, VertexWorld);
pub type TriangleCoordinates = (usize, usize, usize);
pub type Normal = Vector3<f32>;

pub struct CubeVertices {
    pub vertices_local: [VertexLocal; 8],
    pub vertices_world: [VertexWorld; 8],
    pub densities: [f32; 8],
}

impl CubeVertices {
    pub fn edge_indices_to_local_positions(&self, edge: &EdgeIndices) -> EdgeLocalExtremes {
        let a = self.vertices_local[edge.0];
        let b = self.vertices_local[edge.1];

        if a.x < b.x || a.y < b.y || a.z < b.z {
            (a, b)
        } else {
            (b, a)
        }
    }
}

pub trait FluidSnapshot {
    /// Returns an iterator over all particles of the fluid
    fn particles(&self) -> Vec<VertexWorld>; //dyn std::iter::Iterator<Item=Vector3<f32>>;

    /// Returns the number of particles present in the snapshot
    fn len(&self) -> usize;

    /// Returns the position of the particle
    /// * `i` - index of the particle
    fn position(&self, i: usize) -> VertexWorld;

    /// Returns an iterator over all neighbours of the given particle
    /// * `i` - index of the particle
    fn neighbours(&self, i: usize) -> &Vec<usize>; //dyn std::iter::Iterator<Item=Vector3<f32>>;

    /// Returns an iterator over all neighbours of the given particle
    /// * `x` - center of the sphere of research
    fn find_neighbours(&self, x: &VertexWorld) -> Vec<usize>;

    /// Returns the volume of the given particle
    /// * `i` - index of the particle
    fn volume(&self, i: usize) -> f32;

    /// Returns a reference to the kernel used by the simulation
    fn get_kernel(&self) -> &dyn Kernel;

    /// Returns an Axis-Aligned bounding box partition of the space where the liquid is located
    ///
    /// # Arguments
    /// * `min_dist` - Minimum distance between bounding boxes
    ///
    /// # Default Implementation
    /// By default, an iteration over all samples is performed to find the bottom left front and
    /// the top right back vertices of the AABB containing the whole fluid and returns them
    ///
    /// # Optimization
    /// To decrease the cpu/memory consummation, it's better to split the fluid into multiple AABB
    fn aabb(&self, min_dist: f32) -> std::vec::Vec<(Vector3<f32>, Vector3<f32>)> {
        // FIXME: the default implementation is too naive, it'd be better to split the fluid into multiple AABB
        let mut a = Vector3::new(f32::MAX, f32::MAX, f32::MAX); // bottom left front
        let mut b = Vector3::new(f32::MIN, f32::MIN, f32::MIN); // top right back

        self.particles()
            .iter()
            .for_each(|p| {
                a.x = a.x.min(p.x);
                a.y = a.y.min(p.y);
                a.z = a.z.min(p.z);

                b.x = b.x.max(p.x);
                b.y = b.y.max(p.y);
                b.z = b.z.max(p.z);
            });

        vec![(a - Vector3::new(min_dist, min_dist, min_dist), b + Vector3::new(min_dist, min_dist, min_dist))]
    }
}


pub trait FluidSnapshotProvider {
    fn radius(&self) -> f32;
    fn snapshot(&self, radius: f32) -> Box<dyn FluidSnapshot>;
}

