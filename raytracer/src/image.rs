extern crate image;

use std::path::Path;
use nalgebra::Vector3;

pub fn load_image(path: &Path) -> Result<(u32, u32, Vec<Vector3<f32>>), Box<dyn std::error::Error>>
{
    let image = image::open(path)?.to_rgb();

    let width = image.width();
    let height = image.height();

    let mut result = vec![];

    for pix in image.pixels()
    {
        let data = pix;

        result.push(Vector3::new(
            data[0] as f32 / 255.0,
            data[1] as f32 / 255.0,
            data[2] as f32 / 255.0,
        ));
    }

    Ok((width, height, result))
}
