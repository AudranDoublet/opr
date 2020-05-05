use clap::ArgMatches;
use std::path::Path;

use raytracer::Scene;
use std::time::Instant;
use raytracer::scene_config::SceneConfig;

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
    let config = SceneConfig::load(&Path::new(scene))?;

    let (width, height) = (config.camera.width(), config.camera.height());
    println!("width: {}, height: {}", width, height);
    
    let mut scene = Scene::from_config(config, &Path::new("data/materials/white.mtl"))?;

    scene.build(12);

    let pixels;

    timeit!("rendering", pixels = scene.render(width as usize, height as usize, 10, 16));

    pixels.save(&Path::new("result.png"));

    Ok(())
}

