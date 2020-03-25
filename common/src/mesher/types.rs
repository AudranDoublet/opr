use nalgebra::Vector3;

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
    pub fn edge_indices_to_local_position(&self, edge: &EdgeIndices) -> EdgeLocalExtremes {
        (self.vertices_local[edge.0], self.vertices_local[edge.1])
    }
}

pub trait FluidSnapshot {
    /// Returns an iterator over all particles of the fluid
    fn particles(&self) -> std::vec::Vec<Vector3<f32>>; //dyn std::iter::Iterator<Item=Vector3<f32>>;

    /// Returns the density of the fluid at the given position
    /// # Arguments
    /// * `position` - World-Coordinates of the space location where the density must be returned
    fn density_at(&self, position: Vector3<f32>) -> f32;

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
