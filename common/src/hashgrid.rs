use std::collections::{HashMap, HashSet};
use std::iter::FromIterator;

use derivative::*;
use itertools::Itertools;
use nalgebra::Vector3;
use rayon::prelude::*;
use serde_derive::*;

use crate::mesher::types::VertexWorld;

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

#[derive(PartialEq, Debug, Eq, Clone, Copy)]
enum LakeState {
    EXTERNAL = -1,
    JUNCTURE = 0,
    INTERNAL = 1,
}

#[derive(Serialize, Deserialize, Derivative, Clone)]
#[derivative(Debug)]
pub struct HashGrid
{
    cell_size: f32,
    cell_size_sq: f32,
    #[serde(skip_serializing, skip_deserializing)]
    #[derivative(Debug = "ignore")]
    map: HashMap<HashGridKey, Vec<usize>>,
}

impl HashGrid
{
    pub fn new(cell_size: f32) -> HashGrid {
        HashGrid {
            cell_size: cell_size,
            cell_size_sq: cell_size.powi(2),
            map: HashMap::new(),
        }
    }

    pub fn insert(&mut self, particles: &Vec<Vector3<f32>>) {
        let cell_size = self.cell_size;
        let to_update: HashMap<HashGridKey, Vec<usize>> = particles.par_iter().enumerate()
            .map(|(i, p)| (HashGridKey::new(cell_size, *p), i))
            .collect::<Vec<(HashGridKey, usize)>>()
            .into_iter()
            .into_group_map()
            .into_par_iter()
            .map(|(k, v)| {
                let s_i: HashSet<usize> = HashSet::from_iter(v.into_iter());
                (k, s_i.into_iter().collect())
            }).collect();

        self.map.par_extend(
            to_update.into_par_iter()
        );
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

    pub fn update_particles(&mut self, old_positions: &Vec<VertexWorld>, new_positions: &Vec<VertexWorld>) {
        let cell_size = self.cell_size;
        let to_update: Vec<(usize, HashGridKey, HashGridKey)> = old_positions.par_iter().zip(new_positions.par_iter()).enumerate()
            .filter(|(_, (old, new))| *old != *new)
            .map(|(i, (old, new))| {
                (i, HashGridKey::new(cell_size, *old), HashGridKey::new(cell_size, *new))
            }).collect();

        let to_remove: HashMap<HashGridKey, HashSet<usize>> = to_update.par_iter()
            .map(|(i, old, _new)| (*old, *i))
            .collect::<Vec<(HashGridKey, usize)>>()
            .into_iter()
            .into_group_map()
            .into_par_iter()
            .map(|(k, v)| (k, HashSet::from_iter(v.into_iter())))
            .collect();

        let to_add: HashMap<HashGridKey, HashSet<usize>> = to_update.par_iter()
            .map(|(i, _old, new)| (*new, *i))
            .collect::<Vec<(HashGridKey, usize)>>()
            .into_iter()
            .into_group_map()
            .into_par_iter()
            .map(|(k, v)| (k, HashSet::from_iter(v.into_iter())))
            .collect();

        let keys: HashSet<HashGridKey> = HashSet::from_par_iter(to_remove.par_iter().chain(to_add.par_iter()).into_par_iter().map(|(&k, _)| k));

        let new_values: Vec<(HashGridKey, Vec<usize>)> = keys.into_par_iter().map(|k| {
            let mut entry = self.map.get(&k).unwrap_or(&vec![]).clone();

            if let Some(v_to_del) = to_remove.get(&k) {
                entry.retain(|v| v_to_del.contains(v));
            }

            if let Some(v_to_add) = to_add.get(&k) {
                entry.extend(v_to_add.iter());
            }

            let entry: HashSet<usize> = HashSet::from_iter(entry.into_iter());

            (k, entry.into_iter().collect_vec())
        }).collect();

        self.map.par_extend(new_values.into_par_iter());
    }

    fn _get_borders_rec(&self, cell: &HashGridKey, grid: &mut HashMap<HashGridKey, LakeState>) -> LakeState {
        grid.insert(*cell, LakeState::INTERNAL);

        let mut count_non_void_neighbours: u8 = 0;

        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }

                    let neighbour = cell.relative(dx, dy, dz);

                    let neighbour_state = match grid.get(&neighbour) {
                        Some(&state) => state,
                        None => {
                            match self.map.get(&neighbour) {
                                Some(particles) =>
                                    if particles.is_empty() { LakeState::EXTERNAL } else { self._get_borders_rec(&neighbour, grid) }
                                None =>
                                    LakeState::EXTERNAL
                            }
                        }
                    };

                    if neighbour_state == LakeState::EXTERNAL {
                        grid.entry(neighbour).or_insert(LakeState::EXTERNAL);
                    }

                    count_non_void_neighbours += (neighbour_state != LakeState::EXTERNAL) as u8;
                }
            }
        }

        if count_non_void_neighbours == 26 {
            LakeState::INTERNAL
        } else {
            grid.insert(*cell, LakeState::JUNCTURE);
            LakeState::JUNCTURE
        }
    }

    fn convert_into_world_vertex(&self, x: &HashGridKey) -> VertexWorld {
        VertexWorld::new(x.x as f32, x.y as f32, x.z as f32) * self.cell_size
    }

    pub fn get_borders(&self) -> Vec<VertexWorld> {
        let mut result = vec![];

        let mut grid: HashMap<HashGridKey, LakeState> = HashMap::new();

        self.map.iter()
            .filter(|(_k, v)| !v.is_empty())
            .for_each(|(k, _)| {
                if !grid.contains_key(k) {
                    self._get_borders_rec(k, &mut grid);
                }
            });

        result.extend(
            grid.into_iter()
                .filter(|(_, s)| *s != LakeState::INTERNAL)
                .map(|(k, _)| self.convert_into_world_vertex(&k))
        );

        result
    }
}
