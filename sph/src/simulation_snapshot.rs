use nalgebra::Vector3;
use utils::kernels::{Kernel, CubicSpine};

use mesher::types::{FluidSnapshot, VertexWorld};

use search::HashGrid;

pub struct SimulationFluidSnapshot
{
    pub particles: Vec<Vector3<f32>>,
    pub densities: Vec<f32>,
    pub neighbours_struct: HashGrid,
    pub anisotropic_neighbours: Vec<Vec<usize>>,
    pub kernel: CubicSpine,
    pub mass: f32,
}

impl FluidSnapshot for SimulationFluidSnapshot {
    fn len(&self) -> usize {
        self.particles.len()
    }

    fn position(&self, i: usize) -> VertexWorld {
        self.particles[i]
    }

    fn neighbours_anisotropic_kernel(&self, i: usize) -> &Vec<usize> {
        self.anisotropic_neighbours[i].as_ref()
    }

    fn neighbours_kernel(&self, x: &VertexWorld) -> Vec<usize> {
        self.neighbours_struct.find_neighbours(self.len(), &self.particles, *x)
    }

    fn mass(&self, _i: usize) -> f32 {
        self.mass
    }

    fn density(&self, i: usize) -> f32 {
        self.densities[i]
    }

    fn get_kernel(&self) -> &dyn Kernel {
        &self.kernel
    }

    fn get_borders(&self) -> (Vec<Vector3<f32>>, f32) {
        self.neighbours_struct.get_borders()
    }
}
