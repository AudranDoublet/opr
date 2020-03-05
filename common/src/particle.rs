extern crate nalgebra;

use nalgebra::Vector3;

pub struct ParticleNeighbour
{
    id: usize,
}

pub struct Particle
{
    position: Vector3<f32>,
    density: f32,
    neighbours: Vec<ParticleNeighbour>,
}

impl Particle
{
    pub fn new() -> Particle
    {
        Particle {
            position: Vector3::zeros(),
            neighbours: Vec::new(),
            density: 0.0,
        }
    }
}
