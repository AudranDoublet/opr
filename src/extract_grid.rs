extern crate image;

use std::path::Path;
use sph_common::{DiscreteGrid, mesh::Mesh};

use nalgebra::Vector3;

fn rescale(v: usize, s: usize, scale: f32) -> f32 {
    (v as f32) / (s as f32) * scale - scale / 2.
}

/**
 * Extract an image from a grid (for testing purpose)
 */
pub fn extract_grid(path: &Path, mesh: &Mesh, grid: &DiscreteGrid, resolution: (usize, usize), z: f32)
{
    let mut buffer = vec!(0 as u8; resolution.0 * resolution.1 * 3);

    for y in 0..resolution.1
    {
        for x in 0..resolution.0 {
            let i = (x + (resolution.1-y-1)*resolution.0) * 3;
            let pos = Vector3::new(rescale(x, resolution.0, 2.0), rescale(y, resolution.1, 2.0), z);

            /*
            let value = grid.interpolate(1, Vector3::new(rescale(x, resolution.0, 4.0), rescale(y, resolution.1, 4.0), z), false);
            let color = match value {
                Some((value, _grad)) => value * 100. / 0.64,
                //Some(value) if value <= 0.01 => 1.0 + value.clamp(-0.1, 0.1) * 5.,
                //Some(value) if value <= 0.01 => 1.0 + value.clamp(-1.0, 0.0),
                _ => 0.0,
            };

            buffer[i] = (color * 255.0) as u8;
            buffer[i + 1] = 0;
            buffer[i + 2] = 0;
            */

            let (_dist, _) = grid.interpolate_or(0, pos, false);

            let grad = match grid.interpolate(1, pos, true) {
                Some((_, grad)) => grad / 0.1 + Vector3::new(0.5, 0.5, 0.5),
                _ => Vector3::zeros(),
            };

            buffer[i + 0] = (grad.x * 255.0) as u8;
            buffer[i + 1] = (grad.y * 255.0) as u8;
            buffer[i + 2] = (grad.z * 255.0) as u8;
        }
    }

    match image::save_buffer(path, &buffer, resolution.0 as u32, resolution.1 as u32, image::ColorType::Rgb8) {
        Ok(_) => (),
        err => panic!("can't save image: {:?}", err)
    }
}
