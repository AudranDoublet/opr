use std::path::{Path, PathBuf};
use std::fs;

use indicatif::{ProgressBar, ProgressStyle};

use sph_scene::Scene;
use sph_common::Camera;

use nalgebra::Vector3;

extern crate raytracer;
use raytracer::{write_image, Light};
use raytracer::scene_config::*;

fn get_obj(folder: &Path) -> Result<Vec<PathBuf>, Box<dyn std::error::Error>> {
    let mut files: Vec<PathBuf> = fs::read_dir(folder)?
        .filter_map(Result::ok)
        .filter_map(|d| d.path().to_str().and_then(|f| if f.ends_with(".yaml") { Some(d) } else { None }))
        .map(|d| d.path())
        .collect();

    files.sort();

    Ok(files)
}

pub fn pipeline_render(scene: &Scene, input_directory: &Path, dump_directory: &Path) -> Result<(), Box<dyn std::error::Error>> {
    fs::create_dir_all(dump_directory)?;

    let simulations: Vec<PathBuf> = get_obj(input_directory)?;

    let pb = ProgressBar::new(simulations.len() as u64);
    pb.set_style(ProgressStyle::default_bar()
      .template("[{elapsed_precise}] [{per_sec}] [{eta_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7}"));
    pb.tick();

    let lights = vec![
        Light::ambient(Vector3::new(0.3, 0.3, 0.3)),
        Light::directional(Vector3::new(-1., 1., -1.), Vector3::new(1., 1., 1.)),
    ];

    let (width, height) = scene.render_config.resolution;

    for idx in 0..simulations.len() {
        let mut ray_scene = SceneConfig::load(&simulations[idx])?;
        ray_scene.lights = lights.clone();

        let mut render_scene = raytracer::Scene::from_config(ray_scene, &Path::new("data/materials/white.mtl"))?;

        if scene.camera.generate_at_render {
            let dt = (idx as f32) / scene.simulation_config.fps;
            let mut camera = Camera::new(scene.camera.position);
            camera.tick(dt, &mut scene.camera.animation.clone());

            render_scene.setup_camera(camera.position(), camera.up(), camera.forward());
        }

        render_scene.build(12);

        let pixels = render_scene.render(width, height);

        write_image(&dump_directory.join(format!("{:08}.png", idx)), &pixels, width, height);

        pb.inc(1);
    }

    pb.finish_with_message("rendering done");

    Ok(())
}   
