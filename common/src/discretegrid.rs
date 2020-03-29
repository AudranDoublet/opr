extern crate rayon;

use serde_cbor;
use serde_derive::{Deserialize, Serialize};

use nalgebra::Vector3;
use rayon::prelude::*;

use std::fs::File;

#[derive(Debug, Serialize, Deserialize)]
pub struct DiscreteGridFunction {
    weights: Vec<f32>,
    cells: Vec<Vec<u32>>,
}

impl DiscreteGridFunction {
    pub fn interpolate(&self, index: usize, shape: Vec<f32>) -> f32 {
        self.cells[index].iter().zip(shape).fold(0.0, |total, (a, b)| total + self.weights[*a as usize]*b)
    }

    pub fn interpolate_gradient(&self, index: usize, shape: Vec<(f32, f32, f32)>, factor: Vector3<f32>) -> Vector3<f32> {
        factor.component_mul(&self.cells[index].iter().zip(shape).fold(
            Vector3::zeros(),
            |total, (a, b)| total + self.weights[*a as usize]*Vector3::new(b.0, b.1, b.2)
        ))
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct DiscreteGrid {
    domain_min: Vector3<f32>,
    domain_max: Vector3<f32>,
    resolution: Vector3<u32>,
    cell_size: Vector3<f32>,
    cell_count: u32,
    functions: Vec<DiscreteGridFunction>,
}

impl Default for DiscreteGrid {
    fn default() -> Self {
        DiscreteGrid{
            domain_min: Vector3::zeros(),
            domain_max: Vector3::zeros(),
            resolution: Vector3::zeros(),
            cell_size: Vector3::zeros(),
            cell_count: 0,
            functions: vec![]
        }
    }
}

impl DiscreteGrid {
    pub fn load(path: &File) -> Result<DiscreteGrid, Box<dyn std::error::Error>> {
        let result: DiscreteGrid = serde_cbor::from_reader(path)?;

        Ok(result)
    }

    pub fn save(&self, file: &File) -> Result<(), Box<dyn std::error::Error>> {
        serde_cbor::to_writer(file, &self)?;

        Ok(())
    }

    pub fn new(domain_min: Vector3<f32>, domain_max: Vector3<f32>, resolution: Vector3<u32>) -> DiscreteGrid {
        let diff = domain_max - domain_min;

        DiscreteGrid {
            domain_min: domain_min,
            domain_max: domain_max,
            resolution: resolution,
            cell_size: Vector3::new(
                diff.x / resolution.x as f32,
                diff.y / resolution.y as f32,
                diff.z / resolution.z as f32,
            ),
            cell_count: resolution.x * resolution.y * resolution.z,
            functions: Vec::new(),
        }
    }

    pub fn contains(&self, pos: Vector3<f32>) -> bool {
        self.domain_min <= pos && self.domain_max >= pos
    }

    fn compute_edges(&self, ni: &mut Vec<f32>, comp_grad: bool, gd: &mut Vec<(f32, f32, f32)>, x: f32, y: f32, z: f32, order: &dyn Fn(f32, f32, f32) -> (f32, f32, f32)) {
        let coeff = 9. / 64.;
        let edge_fact = coeff * (1. - x*x);

        for i in &[-1., 1.] {
            for j in &[-1., 1.] {
                for k in &[-1./3., 1./3.] {
                    ni.push(edge_fact * (1. + 9.*x*k) * (1. + y*j) * (1. + z*i));

                    if comp_grad {
                        gd.push(order(
                            coeff * (3.*k*(3. - 9.*x*x) - 2.*x) * (1. + y*j) * (1. + z*i),
                            coeff * (1. - x*x) * (1. + 9.*k*x) * j*(1. + z*i),
                            coeff * (1. - x*x) * (1. + 9.*k*x) * i*(1. + y*j),
                        ));
                    }
                }
            }
        }
    }

    fn shape(&self, comp_gradient: bool, x: f32, y: f32, z: f32) -> (Vec<f32>, Vec<(f32, f32, f32)>) {
        let mut ni = Vec::new();
        let mut gd = Vec::new();

        let coeff = 1. / 64.;
        let corner_fact = coeff * (9. * (x*x + y*y + z*z) - 19.);

        for i in &[-1., 1.] {
            for j in &[-1., 1.] {
                for k in &[-1., 1.] {
                    ni.push(corner_fact * (1. + x*k) * (1. + y*j) * (1. + z*i));

                    if comp_gradient {
                        gd.push((
                            coeff * (18.*x + 9.*k*(3.*x*x + y*y + z*z) - 19.*k) * (1. + y*j) * (1. + z*i),
                            coeff * (18.*y + 9.*j*(x*x + 3.*y*y + z*z) - 19.*j) * (1. + x*k) * (1. + z*i),
                            coeff * (18.*z + 9.*i*(x*x + y*y + 3.*z*z) - 19.*i) * (1. + x*k) * (1. + y*j)
                        ));
                    }
                }
            }
        }

        self.compute_edges(&mut ni, comp_gradient, &mut gd, x, y, z, &|a, b, c| (a, b, c));
        self.compute_edges(&mut ni, comp_gradient, &mut gd, y, z, x, &|a, b, c| (c, a, b));
        self.compute_edges(&mut ni, comp_gradient, &mut gd, z, x, y, &|a, b, c| (b, c, a));

        (ni, gd)
    }

    fn index_to_position(&self, i: u32) -> Vector3<f32> {
        let vx = self.resolution.x + 1;
        let vy = self.resolution.y + 1;
        let vz = self.resolution.z + 1;

        let nv = vx * vy * vz;


        let get_coord = |i: u32, special: i32| {
            let j = match special {
                -1 => i,
                _  => i / 2,
            };

            let z = (j / (vx * vy)) as f32;
            let j = j % (vx * vy);

            let y = (j / vx) as f32;
            let x = (j % vx) as f32;

            let mut pos = Vector3::new(x, y, z);

            if special != -1 {
                pos[special as usize] += ((i % 2) + 1) as f32 / 3.;
            }

            pos.component_mul(&self.cell_size)
        };

        self.domain_min + match i {
            i if i < nv     => get_coord(i, -1),
            i if i < 3*nv   => get_coord(i - nv, 0),
            i if i < 5*nv   => get_coord(i - 3*nv, 1),
            i               => get_coord(i - 5*nv, 2),
        }
    }

    pub fn create_function<F: Fn(Vector3<f32>) -> f32 + Send + Sync>(&self, f: F) -> DiscreteGridFunction {
        let count = (self.resolution.x + 1) * (self.resolution.y + 1) * (self.resolution.z + 1);

        let vx = self.resolution.x + 1;
        let vy = self.resolution.y + 1;

        // compute weights for each nodes
        let weights : Vec<f32> = (0..count*7).into_par_iter().map(|i| f(self.index_to_position(i))).collect();
        let cells: Vec<Vec<u32>> = (0..self.cell_count).into_par_iter().map(|i| {
            let z = i / (self.resolution.x * self.resolution.y);
            let i = i % (self.resolution.x * self.resolution.y);
            let y = i / self.resolution.x;
            let x = i % self.resolution.y;

            let mut cell = Vec::new();

            for k in 0..=1 {
                for j in 0..=1 {
                    for i in 0..=1 {
                        cell.push((z + k) * (vy * vx) + (y + j) * vx + (x + i));
                    }
                }
            }

            let offset = count;
            for k in 0..=1 {
                for j in 0..=1 {
                    for i in 0..=1 {
                        let pos = (z + k) * (vy * vx) + (y + j) * vx + x;
                        cell.push(offset + 2*pos + i);
                    }
                }
            }

            let offset = count * 3;
            for i in 0..=1 {
                for k in 0..=1 {
                    for j in 0..=1 {
                        let pos = (z + k) * (vy * vx) + (y) * vx + (x + i);
                        cell.push(offset + 2*pos + j);
                    }
                }
            }

            let offset = count * 5;
            for j in 0..=1 {
                for i in 0..=1 {
                    for k in 0..=1 {
                        let pos = (z) * (vy * vx) + (y + j) * vx + (x + i);
                        cell.push(offset + 2*pos + k);
                    }
                }
            }

            cell
        }).collect();

        DiscreteGridFunction {
            weights: weights,
            cells: cells,
        }
    }

    pub fn add_function(&mut self, func: DiscreteGridFunction) -> usize {
        self.functions.push(func);
        self.functions.len() - 1
    }

    pub fn interpolate(&self, func: usize, pos: Vector3<f32>, gradient: bool) -> Option<(f32, Vector3<f32>)> {
        if !self.contains(pos) {
            return None;
        }

        let spos = (pos - self.domain_min).component_div(&self.cell_size);
        let mut grid_pos = Vector3::new(spos.x as u32, spos.y as u32, spos.z as u32);
        grid_pos.x = grid_pos.x.min(self.resolution.x - 1);
        grid_pos.y = grid_pos.y.min(self.resolution.y - 1);
        grid_pos.z = grid_pos.z.min(self.resolution.z - 1);

        let index = grid_pos.z * self.resolution.y * self.resolution.x + grid_pos.y * self.resolution.x + grid_pos.x;

        let min = self.domain_min + Vector3::new(grid_pos.x as f32, grid_pos.y as f32, grid_pos.z as f32).component_mul(&self.cell_size);
        let max = min + self.cell_size;

        let c0 = Vector3::new(2.0, 2.0, 2.0).component_div(&(max - min));
        let c1 = (max + min).component_div(&(max - min));

        let fpos = c0.component_mul(&pos) - c1;
        let (shape, grad_shape) = self.shape(gradient, fpos[0], fpos[1], fpos[2]);

        let value = self.functions[func].interpolate(index as usize, shape);
        let gradient = match gradient {
            true => self.functions[func].interpolate_gradient(index as usize, grad_shape, c0),
            false => Vector3::zeros(),
        };

        Some((value, gradient))
    }

    pub fn interpolate_or(&self, func: usize, pos: Vector3<f32>, gradient: bool) -> (f32, Vector3<f32>) {
        match self.interpolate(func, pos, gradient) {
            Some(v) => v,
            None => (std::f32::INFINITY, Vector3::zeros()),
        }
    }
}
