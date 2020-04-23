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

        let neighbours = snapshot.neighbours_kernel(x);

        let mut density = 0.;

        neighbours.iter()
            .map(|j| (j, snapshot.density(*j)))
            .filter(|(_, density_j)| *density_j > 0.)
            .for_each(|(j, density_j)| {
                density += snapshot.mass(*j) / density_j * f_approx_density(*j);
            });

        density
    }

    fn to_local(&self, point: &VertexWorld) -> VertexLocal {
        VertexLocal::new(
            (point.x / self.cube_size).ceil() as i32,
            (point.y / self.cube_size).ceil() as i32,
            (point.z / self.cube_size).ceil() as i32,
        )
    }

    fn to_world(&self, point: &VertexLocal) -> VertexWorld {
        nalgebra::convert::<VertexLocal, VertexWorld>(point.clone()) * self.cube_size
    }

    fn get_cube_density(&self, snapshot: &Box<dyn FluidSnapshot>, cache_densities: &mut HashMap<VertexLocal, f32>, x_local: &VertexLocal, x_world: &VertexWorld) -> f32 {
        *cache_densities.entry(*x_local)
            .or_insert_with(|| self.compute_density_at(snapshot, x_world))
    }

    fn generate_cube_vertices(&mut self, snapshot: &Box<dyn FluidSnapshot>, cache_densities: &mut HashMap<VertexLocal, f32>,
                              local: &VertexLocal) -> CubeVertices {
        let vertices_local = [
            local + VertexLocal::new(-1, 0, 0),   // 0
            local + VertexLocal::new(0, 0, 0),    // 1
            local + VertexLocal::new(0, 0, -1),   // 2
            local + VertexLocal::new(-1, 0, -1),  // 3
            local + VertexLocal::new(-1, -1, 0),  // 4
            local + VertexLocal::new(0, -1, 0),   // 5
            local + VertexLocal::new(0, -1, -1),  // 6
            local + VertexLocal::new(-1, -1, -1), // 7
        ];

        let mut vertices_world: [VertexWorld; 8] = [Vector3::zeros(); 8];
        let mut densities: [f32; 8] = [0.; 8];

        for i in 0..8 {
            vertices_world[i] = self.to_world(&vertices_local[i]);
            densities[i] = self.get_cube_density(snapshot, cache_densities, &vertices_local[i], &vertices_world[i])
        }

        CubeVertices { vertices_local, vertices_world, densities }
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
        let (borders, cell_discretization) = snapshot.get_grid().get_borders();

        let iso_value = self.iso_value;
        let interpolation_algorithm = self.interpolation_algorithm;
        let interpolator: Box<dyn Fn(&CubeVertices, &EdgeIndices) -> VertexWorld>
            = Box::new(move |cube_vertices, edge_indices|
            interpolate(interpolation_algorithm, iso_value, cube_vertices, edge_indices));

        let mut mesh = Mesh::new();

        let mut cache_vertices: HashMap<VertexLocal, f32> = HashMap::new();


        borders.iter()
            .for_each(|cube_origin| {
                let (min, max) = (self.to_local(&cube_origin.add_scalar(-cell_discretization)), self.to_local(cube_origin));

                let steps: Vector3<i32> = max - min;

                for dz in 0..=steps.z {
                    for dy in 0..=steps.y {
                        for dx in 0..=steps.x {
                            let local_pos = &min + Vector3::new(dx, dy, dz);
                            let cube_vertices = self.generate_cube_vertices(snapshot, &mut cache_vertices, &local_pos);
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
            });

        mesh
    }

    fn convert_mesh_into_obj(&mut self, mesh: Mesh) -> (Vec<VertexWorld>, Vec<Normal>, Vec<TriangleCoordinates>) {
        (
            mesh.vertices,
            mesh.normals.iter().map(|normals| normals.iter().fold(Vector3::zeros(), |a, b| a + b).normalize()).collect(),
            mesh.triangles
        )
    }

    fn get_anisotropic_kernel_radius(&self, snapshot_provider: &impl FluidSnapshotProvider) -> Option<f32> {
        if self.anisotropicator.is_some() {
            Some(Anisotropicator::compute_radius(snapshot_provider.radius()))
        } else {
            None
        }
    }

    fn update_anisotropicator(&mut self, snapshot: &Box<dyn FluidSnapshot>) {
        if let Some(ref mut anisotropicator) = self.anisotropicator {
            anisotropicator.precompute_positions(snapshot);
        }
    }

    pub fn convert_into_obj(&mut self, snapshot_provider: &impl FluidSnapshotProvider, writer: &mut impl std::io::Write) {
        let an_radius = self.get_anisotropic_kernel_radius(snapshot_provider);
        let snapshot = &snapshot_provider.snapshot(snapshot_provider.radius(), an_radius);
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
