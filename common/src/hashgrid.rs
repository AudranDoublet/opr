use serde_derive::*;
use derivative::*;

use rayon::prelude::*;
use nalgebra::Vector3;

use crate::{Particle, DFSPH};

use chashmap::CHashMap;

#[derive(Hash, Copy, Clone, PartialEq, Eq)]
pub struct HashGridKey
{
    x: isize,
    y: isize,
    z: isize,
}

impl HashGridKey
{
    pub fn new(cell_size: f32, position: Vector3<f32>) -> HashGridKey {
        let coord = |v| match v {
            v if v > 0.0 => 0,
            _ => -1,
        } + (v / cell_size) as isize;

        HashGridKey {
            x: coord(position.x),
            y: coord(position.y),
            z: coord(position.z),
        }
    }

    pub fn relative(&self, x: isize, y: isize, z: isize) -> HashGridKey {
        HashGridKey {
            x: self.x + x,
            y: self.y + y,
            z: self.z + z,
        }
    }
}

#[derive(Serialize, Deserialize)]
#[derive(Derivative)]
#[derivative(Debug)]
pub struct HashGrid
{
    cell_size: f32,
    cell_size_sq: f32,
    #[serde(skip_serializing)]
    #[serde(skip_deserializing)]
    #[derivative(Debug="ignore")]
    map: CHashMap<HashGridKey, Vec<usize>>,
}

impl HashGrid
{
    pub fn new(cell_size: f32) -> HashGrid {
        HashGrid {
            cell_size: cell_size,
            cell_size_sq: cell_size.powi(2),
            map: CHashMap::new(),
        }
    }

    pub fn insert(&mut self, particles: &Vec<Particle>) {
        particles.par_iter().enumerate().for_each(|(i, p)| {
            let position = self.key(p.position);

            if !self.map.contains_key(&position) {
                self.map.insert_new(position, Vec::new());
            }

            if let Some(mut pos) = self.map.get_mut(&position) {
                pos.push(i)
            }
        });
    }

    pub fn key(&self, position: Vector3<f32>) -> HashGridKey {
        HashGridKey::new(self.cell_size, position)
    }

    pub fn find_neighbours(&self, i: usize, dfsph: &DFSPH, position: Vector3<f32>) -> Vec<usize> {
        let key = self.key(position);
        let mut result = Vec::new();

        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if let Some(vec) = self.map.get(&key.relative(dx, dy, dz)) {
                        for &j in vec.iter() {
                            if i != j && dfsph.distance_sq(i, j) < self.cell_size_sq {
                                result.push(j);
                            }
                        }
                    }
                }
            }
        }

        result
    }

    pub fn find_all_neighbours(&self, dfsph: &DFSPH, particles: &Vec<Particle>) -> Vec<Vec<usize>> {
        particles.par_iter().enumerate().map(|(i, p)| {
            self.find_neighbours(i, dfsph, p.position)
        }).collect()
    }

    pub fn update_particles(&self, dt: f32, particles: &mut Vec<Particle>) {
        particles.par_iter_mut().enumerate().for_each(|(i, p)| {
            let old_key = self.key(p.position);

            p.position += dt * p.velocity;
            let new_key = self.key(p.position);

            if old_key != new_key {
                if let Some(mut vec) = self.map.get_mut(&old_key) {
                    vec.retain(|&x| x != i); // FIXME remove vector if empty ?
                }

                if !self.map.contains_key(&new_key) {
                    self.map.insert_new(new_key, Vec::new());
                }

                if let Some(mut vec) = self.map.get_mut(&new_key) {
                    vec.push(i);
                }
            }
        });
    }
}
