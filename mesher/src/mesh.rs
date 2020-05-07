use std::collections::hash_map::Entry;
use std::collections::HashMap;

use crate::types::*;

pub struct Mesh {
    edge_map: HashMap<EdgeLocalExtremes, usize>, // map an edge to its vertex index

    pub vertices: Vec<VertexWorld>,
    pub normals: Vec<Vec<Normal>>,
    pub triangles: Vec<(usize, usize, usize)>,
}

impl Mesh {
    pub fn new() -> Mesh {
        Mesh {
            edge_map: Default::default(),
            vertices: vec![],
            normals: vec![],
            triangles: vec![],
        }
    }

    fn add_edge<'a>(&mut self, cube_vertices: &CubeVertices, e: &EdgeIndices, interpolator: &Box<dyn Fn(&CubeVertices, &EdgeIndices) -> VertexWorld + 'a>) -> usize {
        let idx = self.edge_map.len();
        let entry = self.edge_map.entry(cube_vertices.edge_indices_to_local_positions(e));
        *match entry {
            Entry::Occupied(ref entry) => {
                entry.get()
            }
            Entry::Vacant(entry) => {
                let vertex = interpolator(cube_vertices, e);

                self.vertices.push(vertex);
                self.normals.push(vec![]);

                assert_eq!(self.vertices.len(), idx + 1);
                assert_eq!(self.vertices.len(), self.normals.len());

                entry.insert(idx)
            }
        }
    }

    fn compute_triangle_normal(&self, triangle: &(usize, usize, usize)) -> Normal {
        let a = &self.vertices[triangle.0];
        let b = &self.vertices[triangle.1];
        let c = &self.vertices[triangle.2];

        (c - a).cross(&(b - a))
    }

    fn add_normals(&mut self, triangle: &(usize, usize, usize)) {
        let normal = self.compute_triangle_normal(triangle);

        self.normals[triangle.0].push(normal);
        self.normals[triangle.1].push(normal);
        self.normals[triangle.2].push(normal);
    }

    pub fn add_triangle<'a>(&mut self, cube_vertices: &CubeVertices, e1: &EdgeIndices, e2: &EdgeIndices, e3: &EdgeIndices, interpolator: &Box<dyn Fn(&CubeVertices, &EdgeIndices) -> VertexWorld + 'a>) {
        let triangle = (
            self.add_edge(cube_vertices, e1, interpolator),
            self.add_edge(cube_vertices, e2, interpolator),
            self.add_edge(cube_vertices, e3, interpolator),
        );

        self.add_normals(&triangle);
        self.triangles.push(triangle);
    }
}
