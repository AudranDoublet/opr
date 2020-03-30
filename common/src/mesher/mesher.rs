use std::collections::HashMap;

use nalgebra::Vector3;

use crate::mesher::anisotropication::Anisotropicator;
use crate::mesher::constants;
use crate::mesher::constants::MC_CONFIGS_EDGES;
use crate::mesher::interpolation::{interpolate, InterpolationAlgorithms};
use crate::mesher::mesh::Mesh;
use crate::mesher::types::*;

#[derive(Clone)]
pub struct Mesher {
    iso_value: f32,
    cube_size: f32,
    interpolation_algorithm: InterpolationAlgorithms,
    cache: HashMap<(i32, i32, i32), f32>,
    anisotropicator: Option<Anisotropicator>,
}

impl Mesher {
    pub fn new(iso_value: f32, cube_size: f32, interpolation_algorithm: InterpolationAlgorithms, anisotropicator: Option<Anisotropicator>) -> Mesher {
        Mesher {
            iso_value,
            cube_size,
            interpolation_algorithm,
            cache: HashMap::new(),
            anisotropicator,
        }
    }

    fn compute_density_at(&self, snapshot: &Box<dyn FluidSnapshot>, x: &VertexWorld) -> f32 {
        let kernel = snapshot.get_kernel();

        let f_approx_density = |j: usize| {
            if let Some(anisotropicator) = &self.anisotropicator {
                        let g = anisotropicator.compute_anisotropy(snapshot, j);
                        let r = x - anisotropicator.smoothed_position(j);
                        let p = g * r;

                        kernel.unormalized_apply_on_norm(p.norm(), g.norm())
            } else {
                kernel.apply_on_norm((x - snapshot.position(j)).norm())
            }
        };

        let neighbours = snapshot.find_neighbours(x);

        let mut density = 0.;

        for j in neighbours {
            density += snapshot.volume(j) * f_approx_density(j);
        }

        density
    }

    fn density_at(&mut self, local: &VertexLocal, x: &VertexWorld, snapshot: &Box<dyn FluidSnapshot>) -> f32 {
        let k = (local.x, local.y, local.z);
        if self.cache.contains_key(&k) {
            *self.cache.get(&k).unwrap()
        } else {
            let d = self.compute_density_at(snapshot, x);
            self.cache.insert(k, d);
            d
        }
    }

    fn generate_cube_vertices(&mut self, snapshot: &Box<dyn FluidSnapshot>, origin: VertexWorld, from_local: VertexLocal) -> CubeVertices {
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
            densities[i] = self.density_at(&vertices_local[i], &vertices_world[i], snapshot);
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

    fn to_mesh_cube(&mut self, snapshot: &Box<dyn FluidSnapshot>, min: VertexWorld, max: VertexWorld) -> Mesh {
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

    fn to_mesh_fluid(&mut self, scene: &Box<dyn FluidSnapshot>) -> (Vec<VertexWorld>, Vec<Normal>, Vec<TriangleCoordinates>) {
        let bounding_boxes = scene.aabb(self.cube_size * 5.);

        let mut r_vertices = Vec::new();
        let mut r_normals = Vec::new();
        let mut r_triangles = Vec::new();

        for (min, max) in bounding_boxes {
            let mesh = self.to_mesh_cube(scene, min, max);
            let vertex_offset = r_vertices.len();

            r_vertices.extend(mesh.vertices);
            r_normals.extend(mesh.normals.iter()
                .map(|normals| normals.iter().fold(Vector3::zeros(), |a, b| a + b).normalize()));
            r_triangles.extend(mesh.triangles.iter()
                .map(|vertex_idx| (vertex_idx.0 + vertex_offset, vertex_idx.1 + vertex_offset, vertex_idx.2 + vertex_offset)));
        }

        (r_vertices, r_normals, r_triangles)
    }

    fn find_radius(&self, snapshot_provider: &impl FluidSnapshotProvider) -> f32{
        if self.anisotropicator.is_some() {
            Anisotropicator::compute_radius(snapshot_provider.radius())
        } else {
            snapshot_provider.radius()
        }
    }

    fn update_anisotropicator(&mut self, snapshot: &Box<dyn FluidSnapshot>) {
        if let Some(ref mut anisotropicator) = self.anisotropicator {
            anisotropicator.precompute_positions(snapshot);
        }
    }

    pub fn to_obj(&mut self, snapshot_provider: &impl FluidSnapshotProvider, writer: &mut impl std::io::Write) {
        let radius = self.find_radius(snapshot_provider);
        let snapshot = &snapshot_provider.snapshot(radius);
        self.update_anisotropicator(snapshot);

        let (vertices, normals, triangles) = self.to_mesh_fluid(snapshot);
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
