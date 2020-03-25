use crate::mesher::types::*;

pub trait Interpolator {
    fn interpolate(value: f32, cube_vertices: &CubeVertices, cube_edge: &(usize, usize)) -> VertexWorld;
}

#[derive(Copy, Clone)]
pub enum InterpolationAlgorithms {
    None,
    Linear
}

fn no_interpolation(cube_vertices: &CubeVertices, cube_edge: &EdgeIndices) -> VertexWorld {
    let (p0, _) = (&cube_vertices.vertices_world[cube_edge.0], cube_vertices.densities[cube_edge.0]);
    let (p1, _) = (&cube_vertices.vertices_world[cube_edge.1], cube_vertices.densities[cube_edge.1]);

    (p0 + p1) / 2.
}

fn linear_interpolation(value: f32, cube_vertices: &CubeVertices, cube_edge: &EdgeIndices) -> VertexWorld {
    let (p0, v0) = (&cube_vertices.vertices_world[cube_edge.0], cube_vertices.densities[cube_edge.0]);
    let (p1, v1) = (&cube_vertices.vertices_world[cube_edge.1], cube_vertices.densities[cube_edge.1]);

    if (v0 - value).abs() < 0.001 {
        return *p0;
    } else if (v1 - value).abs() < 0.001 {
        return *p1;
    }

    ((p1 - p0) / (v1 - v0)) * (value - v0) + p0
}

pub fn interpolate(algorithm: InterpolationAlgorithms, value: f32, cube_vertices: &CubeVertices, cube_edge: &EdgeIndices) -> VertexWorld {
    match algorithm {
        InterpolationAlgorithms::None => no_interpolation(cube_vertices, cube_edge),
        InterpolationAlgorithms::Linear => linear_interpolation(value, cube_vertices, cube_edge)
    }
}
