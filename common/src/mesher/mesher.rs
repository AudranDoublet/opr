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
    anisotropicator: Option<Anisotropicator>,
}

impl Mesher {
    pub fn new(iso_value: f32, cube_size: f32, interpolation_algorithm: InterpolationAlgorithms, anisotropicator: Option<Anisotropicator>) -> Mesher {
        Mesher {
            iso_value,
            cube_size,
            interpolation_algorithm,
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

                kernel.unormalized_apply_on_norm(p.norm(), g.determinant())
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

    fn density_at(&mut self, local: &VertexLocal, x: &VertexWorld, snapshot: &Box<dyn FluidSnapshot>, cache: &mut HashMap<VertexLocal, f32>) -> f32 {
        *cache.entry(*local).or_insert_with(|| self.compute_density_at(snapshot, x))
    }

    fn generate_cube_vertices(&mut self, snapshot: &Box<dyn FluidSnapshot>, origin: VertexLocal, cache: &mut HashMap<VertexLocal, f32>) -> CubeVertices {
        let vertices_local = [
            &origin + Vector3::new(-1, 0, 0),   // 0
            &origin + Vector3::new(0, 0, 0),    // 1
            &origin + Vector3::new(0, 0, -1),   // 2
            &origin + Vector3::new(-1, 0, -1),  // 3
            &origin + Vector3::new(-1, -1, 0),  // 4
            &origin + Vector3::new(0, -1, 0),   // 5
            &origin + Vector3::new(0, -1, -1),  // 6
            &origin + Vector3::new(-1, -1, -1), // 7
        ];

        let mut vertices_world: [VertexWorld; 8] = [Vector3::zeros(); 8];
        let mut densities: [f32; 8] = [0.; 8];

        for i in 0..8 {
            vertices_world[i] = self.to_world(vertices_local[i]);
            densities[i] = self.density_at(&vertices_local[i], &vertices_world[i], snapshot, cache);
        }

        CubeVertices { vertices_local, vertices_world, densities }
    }

    fn to_world(&self, point: VertexLocal) -> VertexWorld {
        nalgebra::convert::<VertexLocal, VertexWorld>(point) * self.cube_size
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

    fn compute_mesh(&mut self, snapshot: &Box<dyn FluidSnapshot>) -> Mesh {
        let bounding_boxes = self.discrete_aabb(snapshot);

        let iso_value = self.iso_value;
        let interpolation_algorithm = self.interpolation_algorithm;
        let interpolator: Box<dyn Fn(&CubeVertices, &EdgeIndices) -> VertexWorld> = Box::new(move |cube_vertices, edge_indices| interpolate(interpolation_algorithm, iso_value, cube_vertices, edge_indices));

        let mut mesh = Mesh::new();

        let mut cache_densities: HashMap<VertexLocal, f32> = HashMap::new();
        let mut cache_cubes: HashMap<VertexLocal, CubeVertices> = HashMap::new();

        for (min, max) in &bounding_boxes {
            for z in min.z..=max.z {
                for y in min.y..=max.y {
                    for x in min.x..=max.x {
                        let cube_origin = VertexLocal::new(x, y, z);

                        let cube_vertices = cache_cubes
                            .entry(cube_origin)
                            .or_insert_with(|| self.generate_cube_vertices(snapshot, cube_origin, &mut cache_densities));

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
        }

        mesh
    }

    fn discrete_aabb(&self, snapshot: &Box<dyn FluidSnapshot>) -> Vec<(VertexLocal, VertexLocal)> {
        let bounding_boxes = snapshot.aabb();
        let kernel_radius = snapshot.get_kernel().radius();

        let align_max = |v: &VertexWorld| -> VertexLocal {
            let x = ((v.x + kernel_radius) / self.cube_size).ceil() as i32;
            let y = ((v.y + kernel_radius) / self.cube_size).ceil() as i32;
            let z = ((v.z + kernel_radius) / self.cube_size).ceil() as i32;

            VertexLocal::new(x, y, z)
        };

        let align_min = |v: &VertexWorld| -> VertexLocal {
            let x = ((v.x - kernel_radius) / self.cube_size).floor() as i32;
            let y = ((v.y - kernel_radius) / self.cube_size).floor() as i32;
            let z = ((v.z - kernel_radius) / self.cube_size).floor() as i32;

            VertexLocal::new(x, y, z)
        };


        bounding_boxes.iter().map(|(min, max)| (align_min(min), align_max(max))).collect()
    }

    fn convert_mesh_into_obj(&mut self, mesh: Mesh) -> (Vec<VertexWorld>, Vec<Normal>, Vec<TriangleCoordinates>) {
        (
            mesh.vertices,
            mesh.normals.iter().map(|normals| normals.iter().fold(Vector3::zeros(), |a, b| a + b).normalize()).collect(),
            mesh.triangles
        )
    }

    fn find_radius(&self, snapshot_provider: &impl FluidSnapshotProvider) -> f32 {
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

    pub fn convert_into_obj(&mut self, snapshot_provider: &impl FluidSnapshotProvider, writer: &mut impl std::io::Write) {
        let radius = self.find_radius(snapshot_provider);
        let snapshot = &snapshot_provider.snapshot(radius);
        self.update_anisotropicator(snapshot);

        let mesh = self.compute_mesh(snapshot);
        let (vertices, normals, triangles) = self.convert_mesh_into_obj(mesh);
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
