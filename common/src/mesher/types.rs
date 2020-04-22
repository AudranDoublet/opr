use std::f32;

use nalgebra::Vector3;
use crate::HashGrid;
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
    fn neighbours_anisotropic_kernel(&self, i: usize) -> &Vec<usize>; //dyn std::iter::Iterator<Item=Vector3<f32>>;

    /// Returns an iterator over all neighbours of the given particle
    /// * `x` - center of the sphere of research
    fn neighbours_kernel(&self, x: &VertexWorld) -> Vec<usize>;

    fn mass(&self, i: usize) -> f32;

    fn density(&self, i: usize) -> f32;

    /// Returns a reference to the kernel used by the simulation
    fn get_kernel(&self) -> &dyn Kernel;

    fn get_grid(&self) -> &HashGrid;
}


pub trait FluidSnapshotProvider {
    fn radius(&self) -> f32;
    fn snapshot(&self, radius: f32, anisotropic_radius: Option<f32>) -> Box<dyn FluidSnapshot>;
}

