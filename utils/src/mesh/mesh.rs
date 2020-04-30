use std::error::Error;
use std::path::Path;

use std::collections::HashMap;

use crate::mesh::{Triangle, BoundingSphereHierarchy, tobj};
use crate::{kernels, kernels::Kernel, GaussLegendre};
use crate::DiscreteGrid;

use nalgebra::{Vector3, Matrix3};

pub struct Mesh
{
    bsh: BoundingSphereHierarchy,
    triangles: Vec<Triangle>,
    distance_mult: f32,
    translate: Vector3<f32>,
    bounding_min: Vector3<f32>,
    bounding_max: Vector3<f32>,
}

pub struct MassProperties
{
    pub mass: f32,
    pub center_of_mass: Vector3<f32>,
    pub inertia: Matrix3<f32>,
}

impl Mesh
{
    pub fn load_obj(path: &Path, scale: Vector3<f32>) -> Result<Mesh, Box<dyn Error>> {
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
                                           mesh.positions[i + 2]).component_mul(&scale));
            }

            let mut vertices_normal = vec![];

            for i in (0..mesh.normals.len()).step_by(3)
            {
                vertices_normal.push(Vector3::new(mesh.normals[i + 0],
                                           mesh.normals[i + 1],
                                           mesh.normals[i + 2]));
            }

            if vertices_normal.len() != vertices.len() {
                println!("Drop vertices normals");

                // compute new vertices normals by computing the mean of neighbouring triangles
                // normals
                vertices_normal = vec![Vector3::zeros(); vertices.len()];

                for i in (0..mesh.indices.len()).step_by(3) {
                    let va = &vertices[mesh.indices[i + 0] as usize];
                    let vb = &vertices[mesh.indices[i + 1] as usize];
                    let vc = &vertices[mesh.indices[i + 2] as usize];

                    let normal = (vb - va).cross(&(vc - va));

                    vertices_normal[mesh.indices[i + 0] as usize] += normal;
                    vertices_normal[mesh.indices[i + 1] as usize] += normal;
                    vertices_normal[mesh.indices[i + 2] as usize] += normal;
                }
            }

            let mut edges_normal: HashMap<(usize, usize), Vector3<f32>> = HashMap::new();

            let edge = |a: usize, b: usize| (a.min(b), a.max(b));

            for i in (0..mesh.indices.len()).step_by(3)
            {
                let a = mesh.indices[i + 0] as usize;
                let b = mesh.indices[i + 1] as usize;
                let c = mesh.indices[i + 2] as usize;

                let normal = (vertices[b] - vertices[a]).cross(&(vertices[c] - vertices[a]));

                for e in &[edge(a, b), edge(a, c), edge(b, c)] {
                    edges_normal.insert(*e, edges_normal.get(e).unwrap_or(&Vector3::zeros()) + normal);
                }
            }

            for i in (0..mesh.indices.len()).step_by(3)
            {
                let a = mesh.indices[i + 0] as usize;
                let b = mesh.indices[i + 1] as usize;
                let c = mesh.indices[i + 2] as usize;

                triangles.push(
                    Triangle::new(vertices[a],
                                  vertices[b],
                                  vertices[c],
                                  vertices_normal[a].normalize(),
                                  vertices_normal[b].normalize(),
                                  vertices_normal[c].normalize(),
                                  edges_normal[&edge(a, b)],
                                  edges_normal[&edge(a, c)],
                                  edges_normal[&edge(b, c)],
                ));
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
            translate: Vector3::zeros(),
            triangles: triangles,
            distance_mult: 1.0,
            bounding_min: Vector3::new(min[0], min[1], min[2]),
            bounding_max: Vector3::new(max[0], max[1], max[2]),
        })
    }

    /**
     * Invert mesh distances
     */
    pub fn invert(&mut self) {
        self.distance_mult = -self.distance_mult;
    }

    /**
     * Compute the minimal (abs) signed distance between the point and the mesh
     */
    pub fn minimal_signed_distance(&self, p: Vector3<f32>) -> f32 {
        self.bsh.minimal_signed_distance(p + self.translate) * self.distance_mult
    }

    pub fn set_translate(&mut self, p: Vector3<f32>) {
        self.translate = p
    }

    /**
     * Compute signed distance field
     */
    pub fn compute_sdf(&self, grid: &mut DiscreteGrid) {
        let func = grid.create_function(|p| self.minimal_signed_distance(p));
        grid.add_function(func);
    }

    pub fn boundings(&self) -> (Vector3<f32>, Vector3<f32>) {
        (self.bounding_min - self.translate, self.bounding_max - self.translate)
    }

    /**
     * Compute volume field from signed distance field
     */
    pub fn compute_volume(&self, grid: &mut DiscreteGrid, kernel: &kernels::CubicSpine) {
        let legendre = GaussLegendre::init(16);
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

    /**
     * Returns mass properties of the mesh
     * Source: https://www.geometrictools.com/Documentation/PolyhedralMassProperties.pdf
     */
    pub fn compute_mass_properties(&self, density: f32) -> MassProperties {
        let mut intg: [f32; 10] = [0.0; 10];
        let mult: [f32; 10] = [1./6., 1./24., 1./24., 1./24., 1./60., 1./60., 1./60., 1./120., 1./120., 1./120.];

        for triangle in &self.triangles {
            triangle.triangle_mass_properties(&mut intg);
        }

        for i in 0..10 {
            intg[i] *= mult[i] * density;
        }

        let mass = intg[0];

        let cm = Vector3::new(
            intg[1] / mass, intg[2] / mass, intg[3] / mass
        );

        let xx = intg[5] + intg[6] - mass*(cm.y*cm.y + cm.z*cm.z);
        let yy = intg[4] + intg[5] - mass*(cm.x*cm.x + cm.y*cm.y);
        let zz = intg[4] + intg[5] - mass*(cm.x*cm.x + cm.y*cm.y);
        let xy = mass*cm.x*cm.y - intg[7];
        let yz = mass*cm.y*cm.z - intg[8];
        let xz = mass*cm.z*cm.x - intg[9];

        let inertia = Matrix3::new(
            xx, xy, xz,
            xy, yy, yz,
            xz, yz, zz,
        );

        MassProperties {
            mass: mass,
            center_of_mass: cm,
            inertia: inertia,
        }
    }
}
