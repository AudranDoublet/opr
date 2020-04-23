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

pub fn write_image(path: &Path, buff: &[Vector3<f32>], width: usize, height: usize)
{
    let mut buffer = vec!(0 as u8; width * height * 3);
    let mut i = 0;

    for v in buff
    {
        buffer[i] = (v.x * 255.0) as u8;
        buffer[i + 1] = (v.y * 255.0) as u8;
        buffer[i + 2] = (v.z * 255.0) as u8;
        i += 3;
    }

    match image::save_buffer(path, &buffer, width as u32, height as u32, image::ColorType::Rgb8) {
        Ok(_) => (),
        err => panic!("can't save image: {:?}", err)
    }
}
