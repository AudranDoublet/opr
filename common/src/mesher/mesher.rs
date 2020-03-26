use nalgebra::Vector3;

use crate::mesher::constants;
use crate::mesher::constants::MC_CONFIGS_EDGES;
use crate::mesher::interpolation::{interpolate, InterpolationAlgorithms};
use crate::mesher::mesh::Mesh;
use crate::mesher::types::*;

pub struct Mesher {
    iso_value: f32,
    cube_size: f32,
    interpolation_algorithm: InterpolationAlgorithms,
}

impl Mesher {
    pub fn new(iso_value: f32, cube_size: f32, interpolation_algorithm: InterpolationAlgorithms) -> Mesher {
        Mesher {
            iso_value,
            cube_size,
            interpolation_algorithm,
        }
    }

    fn generate_cube_vertices(&self, snapshot: &impl FluidSnapshot, origin: VertexWorld, from_local: VertexLocal) -> CubeVertices {
        let vertices_local = [
            &from_local + Vector3::new(-1, 0, 0),   // 0
            &from_local + Vector3::new(0, 0, 0),    // 1
            &from_local + Vector3::new(0, 0, -1),   // 2
            &from_local + Vector3::new(-1, 0, -1),  // 3
            &from_local + Vector3::new(-1, -1, 0),  // 4
            &from_local + Vector3::new(0, -1, 0),   // 5
            &from_local + Vector3::new(0, -1, -1),  // 6
            &from_local + Vector3::new(-1, -1, -1), // 7
        ];

        let mut vertices_world: [VertexWorld; 8] = [Vector3::zeros(); 8];
        let mut densities: [f32; 8] = [0.; 8];

        for i in 0..8 {
            vertices_world[i] = self.to_world(&origin, vertices_local[i]);
            densities[i] = snapshot.density_at(
                vertices_world[i]
            );
        }

        CubeVertices { vertices_local, vertices_world, densities }
    }

    fn to_world(&self, origin: &VertexWorld, point: VertexLocal) -> VertexWorld {
        origin + nalgebra::convert::<VertexLocal, VertexWorld>(point) * self.cube_size
    }

    fn search_configuration(&self, cube_vertices: &CubeVertices) -> usize {
        let mut index = 0;

        for i in 0..cube_vertices.densities.len() {
            index |= match cube_vertices.densities[i] {
                v if v <= self.iso_value => 1,
                _ => 0
            } << i;
        }

        index
    }

    fn to_mesh_cube(&self, snapshot: &impl FluidSnapshot, min: VertexWorld, max: VertexWorld) -> Mesh {
        let iso_value = self.iso_value;
        let interpolation_algorithm = self.interpolation_algorithm;
        let interpolator: Box<dyn Fn(&CubeVertices, &EdgeIndices) -> VertexWorld> = Box::new(move |cube_vertices, edge_indices| interpolate(interpolation_algorithm, iso_value, cube_vertices, edge_indices));

        let mut mesh = Mesh::new();

        let origin = min;

        let step_z = ((max.z - min.z) / self.cube_size).ceil() as i32;
        let step_y = ((max.y - min.y) / self.cube_size).ceil() as i32;
        let step_x = ((max.x - min.x) / self.cube_size).ceil() as i32;

        for dz in 0..=step_z {
            for dy in 0..=step_y {
                for dx in 0..=step_x {
                    let cube_vertices = self.generate_cube_vertices(snapshot, origin, Vector3::new(dx, dy, dz));
                    let config_id = self.search_configuration(&cube_vertices);

                    for triangle in &constants::MC_CONFIGS_TRIANGLES[config_id] {
                        if triangle[0] == -1 {
                            break;
                        }

                        mesh.add_triangle(
                            &cube_vertices,
                            &MC_CONFIGS_EDGES[triangle[0] as usize],
                            &MC_CONFIGS_EDGES[triangle[1] as usize],
                            &MC_CONFIGS_EDGES[triangle[2] as usize],
                            &interpolator,
                        );
                    }
                }
            }
        }

        mesh
    }

    fn to_mesh_fluid(&self, scene: &impl FluidSnapshot) -> (Vec<VertexWorld>, Vec<Normal>, Vec<TriangleCoordinates>) {
        let bounding_boxes = scene.aabb(self.cube_size);

        let mut r_vertices = Vec::new();
        let mut r_normals = Vec::new();
        let mut r_triangles = Vec::new();

        for (min, max) in bounding_boxes {
            let mesh = self.to_mesh_cube(scene, min, max);
            let vertex_offset = r_vertices.len();

            r_vertices.extend(mesh.vertices);
            r_normals.extend(mesh.normals.iter()
                .map(|normals| normals.iter().fold(Vector3::zeros(), |a, b| a + b).normalize() ));
            r_triangles.extend(mesh.triangles.iter()
                .map(|vertex_idx| (vertex_idx.0 + vertex_offset, vertex_idx.1 + vertex_offset, vertex_idx.2 + vertex_offset)));
        }

        (r_vertices, r_normals, r_triangles)
    }

    pub fn to_obj(&self, scene: &impl FluidSnapshot, writer: &mut impl std::io::Write) {
        let (vertices, normals, triangles) = self.to_mesh_fluid(scene);
        let error_msg_on_write = "unable to write mesh info";

        vertices.iter().for_each(|v| {
            write!(writer, "v {} {} {}\n", v.x, v.y, v.z).expect(&error_msg_on_write);
        });

        normals.iter().for_each(|vn| {
            write!(writer, "vn {} {} {}\n", vn.x, vn.y, vn.z).expect(&error_msg_on_write);
        });

        triangles.iter().for_each(|t| {
            write!(writer, "f {}//{} {}//{} {}//{}\n", t.0 + 1, t.0 + 1, t.1 + 1, t.1 + 1, t.2 + 1, t.2 + 1).expect(&error_msg_on_write);
        });
    }
}
