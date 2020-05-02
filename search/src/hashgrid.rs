use dashmap::DashMap;

use nalgebra::Vector3;
use rayon::prelude::*;

use std::collections::HashMap;

#[derive(Hash, Debug, Copy, Clone, PartialEq, Eq)]
pub struct HashGridKey
{
    x: isize,
    y: isize,
    z: isize,
}

impl HashGridKey
{
    pub fn new(cell_size: f32, position: Vector3<f32>) -> HashGridKey {
        let coord = |v: f32| (v / cell_size).ceil() as isize;

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

#[derive(PartialEq, Debug, Eq, Clone, Copy)]
enum LakeState {
    EXTERNAL = -1,
    JUNCTURE = 0,
    INTERNAL = 1,
}

pub struct HashGrid
{
    cell_size: f32,
    cell_size_sq: f32,
    map: DashMap<HashGridKey, Vec<usize>>,
}

impl Default for HashGrid {
    fn default() -> HashGrid {
        HashGrid::new(0.02)
    }
}

impl HashGrid
{
    pub fn new(cell_size: f32) -> HashGrid {
        HashGrid {
            cell_size: cell_size,
            cell_size_sq: cell_size.powi(2),
            map: DashMap::new(),
        }
    }

    pub fn insert(&mut self, particles: &Vec<Vector3<f32>>) {
        particles.par_iter().enumerate().for_each(|(i, &p)| {
            let position = self.key(p);
            self.map.entry(position).or_insert(Vec::new()).push(i);
        });
    }

    pub fn key(&self, position: Vector3<f32>) -> HashGridKey {
        HashGridKey::new(self.cell_size, position)
    }

    pub fn find_neighbours(&self, i: usize, positions: &Vec<Vector3<f32>>, position: Vector3<f32>) -> Vec<usize> {
        let key = self.key(position);
        let mut result = Vec::new();

        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if let Some(vec) = self.map.get(&key.relative(dx, dy, dz)) {
                        for &j in vec.iter() {
                            if i != j && (position - positions[j]).norm_squared() < self.cell_size_sq {
                                result.push(j);
                            }
                        }
                    }
                }
            }
        }

        result
    }

    pub fn find_all_neighbours(&self, particles: &Vec<Vector3<f32>>) -> Vec<Vec<usize>> {
        particles.par_iter().enumerate().map(|(i, p)| {
            self.find_neighbours(i, particles, *p)
        }).collect()
    }

    pub fn update_particles(&mut self, old_positions: &Vec<Vector3<f32>>, new_positions: &Vec<Vector3<f32>>) {
        let cell_size= self.cell_size;
        old_positions.par_iter().zip(new_positions.par_iter()).enumerate()
            .filter(|(_, (old, new))| *old != *new)
            .map(|(i, (&old, &new))| (i, (HashGridKey::new(cell_size, old), HashGridKey::new(cell_size, new))))
            .for_each(|(i, (old, new))| {
                if let Some(mut vec) = self.map.get_mut(&old) {
                    vec.retain(|&x| x != i); // FIXME: delete empty vector
                }

                self.map.entry(new).or_insert(vec![]).push(i);
            })
    }

    fn _state(&self, cell: &HashGridKey) -> LakeState {
        match self.map.get(&cell) {
            Some(particles) if particles.is_empty() => LakeState::EXTERNAL,
            None => LakeState::EXTERNAL,
            _ => LakeState::INTERNAL,
        }
    }

    fn _get_borders_rec(&self, cell: &HashGridKey, grid: &mut HashMap<HashGridKey, LakeState>) {
        let mut state = self._state(cell);

        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }

                    let neighbour_state = self._state(&cell.relative(dx, dy, dz));

                    if neighbour_state != state {
                        state = LakeState::JUNCTURE;
                    }
                }
            }
        }

        grid.insert(*cell, state);
    }

    pub fn cell_to_world(&self, v: &HashGridKey) -> Vector3<f32> {
        (Vector3::new(v.x as f32, v.y as f32, v.z as f32)) * self.cell_size
    }

    pub fn coord_to_world(&self, v: &Vector3<i64>) -> Vector3<f32> {
        (Vector3::new(v.x as f32, v.y as f32, v.z as f32)) * self.cell_size
    }

    pub fn get_borders(&self) -> (Vec<Vector3<f32>>, f32) {
        let mut result = vec![];

        let mut grid: HashMap<HashGridKey, LakeState> = HashMap::new();

        self.map.iter()
            .filter(|v| !v.is_empty())
            .for_each(|v| {
                let k = v.key();
                if !grid.contains_key(k) {
                    self._get_borders_rec(k, &mut grid);
                }
            });

        result.par_extend(
            grid.into_par_iter()
                .filter(|(_, s)| *s != LakeState::INTERNAL)
                .map(|(k, _)| self.cell_to_world(&k))
        );

        (result, self.cell_size)
    }
}
