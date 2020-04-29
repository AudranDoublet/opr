use clap::ArgMatches;
use std::path::Path;

use std::time::Instant;
use raytracer::Scene;

macro_rules! timeit {
    ($name:expr, $code:expr) => ({
        let now = Instant::now();
        let result = $code;

        println!("{} : {}ms", $name, now.elapsed().as_micros() as f32 / 1000.);

        result
    })
}

pub fn main_render(args: &ArgMatches) -> Result<(), Box<dyn std::error::Error>> {
    let scene = Path::new(args.value_of("scene").unwrap());

    let result_width = 512;
    let result_height = 512;

    let mut scene = Scene::from_file(&Path::new(scene), &Path::new("data/materials/white.mtl"))?;
    scene.build(12);

    let pixels;

    timeit!("rendering", pixels = scene.render(result_width, result_height, 10, 16));

    pixels.save(&Path::new("result.png"));

    Ok(())
}

