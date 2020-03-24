extern crate tobj;

use std::error::Error;
use std::path::Path;

use crate::mesh::{Triangle, BoundingSphereHierarchy};
use crate::{utils, kernels, Kernel, DiscreteGrid};
use nalgebra::Vector3;

pub struct Mesh
{
    bsh: BoundingSphereHierarchy,
    pub bounding_min: Vector3<f32>,
    pub bounding_max: Vector3<f32>,
}

impl Mesh
{
    pub fn load_obj(path: &Path) -> Result<Mesh, Box<dyn Error>> {
        let (models, _materials) = tobj::load_obj(path)?;

        let mut triangles = Vec::new();

        for model in models.iter()
        {
            let mesh = &model.mesh; 

            let mut vertices = vec![];

            for i in (0..mesh.positions.len()).step_by(3)
            {
                vertices.push(Vector3::new(mesh.positions[i + 0],
                                           mesh.positions[i + 1],
                                           mesh.positions[i + 2]));
            }

            let mut vertices_normal = vec![];

            for i in (0..mesh.normals.len()).step_by(3)
            {
                vertices_normal.push(Vector3::new(mesh.normals[i + 0],
                                           mesh.normals[i + 1],
                                           mesh.normals[i + 2]));
            }

            for i in (0..mesh.indices.len()).step_by(3)
            {
                let a = mesh.indices[i + 0] as usize;
                let b = mesh.indices[i + 1] as usize;
                let c = mesh.indices[i + 2] as usize;

                if vertices_normal.len() == 0 {
                    triangles.push(
                        Triangle::new_no_normals(
                            vertices[a],
                            vertices[b],
                            vertices[c],
                    ));
                } else {
                    triangles.push(
                        Triangle::new(vertices[a],
                                      vertices[b],
                                      vertices[c],
                                      vertices_normal[a],
                                      vertices_normal[b],
                                      vertices_normal[c],
                    ));
                }
            }
        }

        let mut min = vec![std::f32::INFINITY; 3];
        let mut max = vec![std::f32::NEG_INFINITY; 3];

        for t in &triangles {
            for i in 0..3 {
                min[i] = min[i].min(t.min(i));
                max[i] = max[i].max(t.max(i));
            }
        }

        Ok(Mesh {
            bsh: BoundingSphereHierarchy::new(&mut triangles, 15),
            bounding_min: Vector3::new(min[0], min[1], min[2]),
            bounding_max: Vector3::new(max[0], max[1], max[2]),
        })
    }

    /**
     * Compute the minimal (abs) signed distance between the point and the mesh
     */
    pub fn minimal_signed_distance(&self, p: Vector3<f32>) -> f32 {
        self.bsh.minimal_signed_distance(p)
    }

    /**
     * Compute signed distance field
     */
    pub fn compute_sdf(&self, grid: &mut DiscreteGrid) {
        let func = grid.create_function(|p| self.minimal_signed_distance(p));
        grid.add_function(func);
    }

    /**
     * Compute volume field from signed distance field
     */
    pub fn compute_volume(&self, grid: &mut DiscreteGrid, kernel: &kernels::CubicSpine) {
        let legendre = utils::GaussLegendre::init(16);
        let min = Vector3::new(-kernel.radius(), -kernel.radius(), -kernel.radius()); 
        let max = -min;

        let c0 = kernel.apply_on_norm(0.0);

        let func = grid.create_function(|p| {
            let (dist, _) = grid.interpolate_or(0, p, false);
            if dist > 2. * kernel.radius() {
                return 0.0;
            }

            let func = |xi: Vector3<f32>| {
                if xi.norm_squared() > kernel.radius_sq() {
                    return 0.0;
                }

                match grid.interpolate(0, p + xi, false) {
                    Some((v, _)) if v < 0.0 => 1.0 - v,
                    Some((v, _)) => kernel.apply_on_norm(v) / c0,
                    _ => 0.0,
                }
            };

            legendre.integrate(min, max, func) * 0.8
        });

        grid.add_function(func);
    }
}
