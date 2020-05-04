use nalgebra::Vector3;
use crate::Particles;
use search::HashGrid;
use utils::kernel::CubicSpine;

pub trait FragmentShader: Sync + Send {
    fn apply(&self, position: &Vector3<f32>, color: &Vector3<f32>) -> Vector3<f32>;
}

pub struct FoamFragmentShader {
    positions: Vec<Vector3<f32>>,
    grid: HashGrid,
    radius: f32,
    color: Vector3<f32>,
    density_scaling_factor: f32,
}

fn weighting(r_norm: f32, h: f32) -> f32 {
    if r_norm <= h {
        1.0 - r_norm / h
    } else {
        0.0
    }
}

impl FoamFragmentShader {
    pub fn new(foam: Particles, color: Vector3<f32>, radius: f32, density_scaling_factor: f32) -> Self {
        let mut grid = HashGrid::new(radius);
        grid.insert(foam.get_positions());

        FoamFragmentShader {
            // FIXME: should avoid costly cloning of the particles positions
            positions: foam.get_positions().clone(),
            // FIXME-END
            grid,
            radius,
            color,
            density_scaling_factor,
        }
    }

    pub fn compute_density(&self, x: &Vector3<f32>) -> f32 {
        let neighbours = self.grid.find_neighbours(self.positions.len(), &self.positions, *x);

        self.density_scaling_factor * 
            neighbours
            .iter()
            .map(|&j| weighting((&self.positions[j] - x).norm(), self.radius)).fold(0., |a, b| a + b)
    }
}

impl FragmentShader for FoamFragmentShader {
    fn apply(&self, position: &Vector3<f32>, color: &Vector3<f32>) -> Vector3<f32> {
        let density = self.compute_density(position);

        let l = (-density).exp();

        (1.0 - l) * &self.color + l * color
    }
}

