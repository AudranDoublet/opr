use std::collections::HashMap;

use nalgebra::Vector3;

use crate::constants;

pub trait FluidSnapshot {
    /// Returns an iterator over all particles of the fluid
    fn particles(&self) -> std::vec::Vec<Vector3<f32>>; //dyn std::iter::Iterator<Item=Vector3<f32>>;

    /// Returns the density of the fluid at the given position
    /// # Arguments
    /// * `position` - World-Coordinates of the space location where the density must be returned
    fn density_at(&self, position: Vector3<f32>) -> f32;

    /// Returns an Axis-Aligned bounding box partition of the space where the liquid is located
    ///
    /// # Default Implementation
    /// By default, an iteration over all samples is performed to find the bottom left front and
    /// the top right back vertices of the AABB containing the whole fluid and returns them
    ///
    /// # Optimization
    /// To decrease the cpu/memory consummation, it's better to split the fluid into multiple AABB
    fn aabb(&self) -> std::vec::Vec<(Vector3<f32>, Vector3<f32>)> {
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

        vec![(a, b)]
    }
}

pub type Vertex = Vector3<f32>;
pub type Triangle = (Vertex, Vertex, Vertex);
pub type TriangleCoordinates = (usize, usize, usize);
pub type Normal = Vector3<f32>;

pub struct Mesher {
    iso_value: f32,
    cube_size: f32,
}

impl Mesher {
    pub fn new(iso_value: f32, cube_size: f32) -> Mesher {
        Mesher {
            iso_value,
            cube_size,
        }
    }

    fn generate_cube_vertices(from_local: Vector3<i32>) -> [Vector3<i32>; 8] {
        [
            &from_local + Vector3::new(-1, 0, 0),   // 0
            &from_local + Vector3::new(0, 0, 0),    // 1
            &from_local + Vector3::new(0, 0, -1),   // 2
            &from_local + Vector3::new(-1, 0, -1),  // 3
            &from_local + Vector3::new(-1, -1, 0),  // 4
            &from_local + Vector3::new(0, -1, 0),   // 5
            &from_local + Vector3::new(0, -1, -1),  // 6
            &from_local + Vector3::new(-1, -1, -1), // 7
        ]
    }

    fn generate_cube_edges(points: &[Vector3<i32>; 8]) -> [Vector3<i32>; 12] {
        [
            &points[0] + &points[1],
            &points[1] + &points[2],
            &points[2] + &points[3],
            &points[3] + &points[0],
            &points[4] + &points[5],
            &points[5] + &points[6],
            &points[6] + &points[7],
            &points[7] + &points[4],
            &points[0] + &points[4],
            &points[1] + &points[5],
            &points[2] + &points[6],
            &points[3] + &points[7],
        ]
    }

    fn to_world(&self, origin: &Vertex, point: Vector3<i32>) -> Vertex {
        origin + nalgebra::convert::<Vector3<i32>, Vertex>(point) * self.cube_size
    }

    fn edges_normal_world(&self, origin: &Vertex, a: Vector3<i32>, b: Vector3<i32>, c: Vector3<i32>) -> Vector3<f32> {
        let a: Vector3<f32> = origin + nalgebra::convert::<Vector3<i32>, Vertex>(a) / 2. * self.cube_size;
        let b: Vector3<f32> = origin + nalgebra::convert::<Vector3<i32>, Vertex>(b) / 2. * self.cube_size;
        let c: Vector3<f32> = origin + nalgebra::convert::<Vector3<i32>, Vertex>(c) / 2. * self.cube_size;

        (c - a).cross(&(b - a)).normalize()
    }

    fn search_configuration(&self, scene: &impl FluidSnapshot, origin: &Vertex, points: &[Vector3<i32>; 8]) -> usize {
        let mut index = 0;

        for i in 0..points.len() {
            let pos = self.to_world(origin, points[i]);
            let density = scene.density_at(pos);
            index |= match density {
                v if v >= self.iso_value => 1,
                _ => 0
            } << i;
        }

        index
    }

    fn to_mesh_cube(&self, scene: &impl FluidSnapshot, min: Vertex, max: Vertex, vertex_start_idx: usize) -> (Vec<Vertex>, Vec<Normal>, Vec<TriangleCoordinates>) {
        let mut vertices_index = HashMap::new();
        let mut vertices_normals = Vec::new();

        let origin = &min;

        let mut r_triangles = Vec::new();

        let step_z = ((max.z - min.z) / self.cube_size).ceil() as i32;
        let step_y = ((max.y - min.y) / self.cube_size).ceil() as i32;
        let step_x = ((max.x - min.x) / self.cube_size).ceil() as i32;

        for dz in 0..=step_z {
            for dy in 0..=step_y {
                for dx in 0..=step_x {
                    let points = Mesher::generate_cube_vertices(Vector3::new(dx, dy, dz));
                    let edges = Mesher::generate_cube_edges(&points);
                    let idx_config = self.search_configuration(scene, origin, &points);

                    for triangle in &constants::MC_CONFIGS[idx_config] {
                        if triangle[0] == -1 {
                            break;
                        }

                        let triangle_vertices = [edges[triangle[0] as usize], edges[triangle[1] as usize], edges[triangle[2] as usize]];
                        let mut triangle_vertices_idx: [usize; 3] = [0, 0, 0];

                        let normal = self.edges_normal_world(&origin,
                                                             triangle_vertices[0],
                                                             triangle_vertices[1],
                                                             triangle_vertices[2]);

                        for i in 0..triangle_vertices.len() {
                            let v = &triangle_vertices[i];
                            if !vertices_index.contains_key(v) {
                                vertices_index.insert(*v, vertex_start_idx + vertices_index.len() as usize);
                                vertices_normals.push(Vec::new());

                                debug_assert_eq!(vertices_normals.len(), vertices_index.len());
                            }

                            triangle_vertices_idx[i] = *vertices_index.get(v).expect("ntm rust");
                            vertices_normals[triangle_vertices_idx[i]].push(normal);
                        }

                        r_triangles.push((triangle_vertices_idx[0], triangle_vertices_idx[1], triangle_vertices_idx[2]));
                    }
                }
            }
        }

        let mut r_vertices = vec![Vector3::zeros(); vertices_index.len()];
        for (v, idx) in vertices_index {
            r_vertices[idx] = self.to_world(origin, v);
        }

        let mut r_normals = Vec::with_capacity(vertices_normals.len());
        for i in 0..vertices_normals.len() {
            r_normals.push(
                vertices_normals[i].iter().fold(Vector3::zeros(), |x, y| x + *y) / vertices_normals[i].len() as f32
            );
        }

        (r_vertices, r_normals, r_triangles)
    }

    fn to_mesh_fluid(&self, scene: &impl FluidSnapshot) -> (Vec<Vertex>, Vec<Normal>, Vec<TriangleCoordinates>) {
        let bounding_boxes = scene.aabb();

        let mut r_vertices = Vec::new();
        let mut r_normals = Vec::new();
        let mut r_triangles = Vec::new();

        for (min, max) in bounding_boxes {
            let (v, n, t) = self.to_mesh_cube(scene, min, max, r_vertices.len());
            r_vertices.extend(v);
            r_normals.extend(n);
            r_triangles.extend(t);
        }

        (r_vertices, r_normals, r_triangles)
    }

    pub fn to_obj(&self, scene: &impl FluidSnapshot, writer: &mut impl std::io::Write) {
        let (vertices, normals, triangles) = self.to_mesh_fluid(scene);

        vertices.iter().for_each(|v| {
            write!(writer, "v {} {} {}\n", v.x, v.y, v.z).expect("ntm rust");
        });

        normals.iter().for_each(|vn| {
            write!(writer, "vn {} {} {}\n", vn.x, vn.y, vn.z).expect("ntm rust");
        });

        triangles.iter().for_each(|t| {
            write!(writer, "f {} {} {}\n", t.0 + 1, t.1 + 1, t.2 + 1).expect("ntm rust");
        });
    }
}
