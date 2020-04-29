use nalgebra::Vector3;

use std::path::{Path, PathBuf};
use std::fs;

use indicatif::{ProgressBar, ProgressStyle};

use sph_scene::Scene;

use sph::Simulation;
use mesher::interpolation::InterpolationAlgorithms;
use mesher::Mesher;
use mesher::anisotropication::Anisotropicator;

use raytracer::{scene_config, Camera};

use rayon::prelude::*;

fn get_simulation_dumps_paths(folder: &Path) -> Result<Vec<PathBuf>, Box<dyn std::error::Error>> {
    let mut files: Vec<PathBuf> = fs::read_dir(folder)?
        .filter_map(Result::ok)
        .filter_map(|d| d.path().to_str().and_then(|f| if f.ends_with(".sim.bin") { Some(d) } else { None }))
        .map(|d| d.path())
        .collect();

    files.sort();

    Ok(files)
}

pub fn pipeline_polygonize(scene: &Scene, input_directory: &Path, dump_directory: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let interpolation_algorithm = match !scene.meshing_config.enable_interpolation {
        true => InterpolationAlgorithms::None,
        false => InterpolationAlgorithms::Linear,
    };

    let anisotropicator = match scene.meshing_config.enable_anisotropication {
        true => Some(Anisotropicator::new(0.9, 5, 8., 1400., 0.5)),
        false => None
    };

    let mesher = Mesher::new(scene.meshing_config.iso_value, scene.meshing_config.cube_size, interpolation_algorithm, anisotropicator);

    fs::create_dir_all(dump_directory)?;

    let simulations: Vec<PathBuf> = get_simulation_dumps_paths(input_directory)?;

    let pb = ProgressBar::new(simulations.len() as u64);
    pb.set_style(ProgressStyle::default_bar()
      .template("[{elapsed_precise}] [{per_sec}] [{eta_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7}"));
    pb.tick();

    simulations.par_iter().enumerate().for_each(|(idx, path)| {
        let simulation  = Simulation::load(&path).unwrap();
        let path        = &dump_directory.join(format!("{:08}.obj", idx));
        let path_yaml   = dump_directory.join(format!("{:08}.yaml", idx));
        let buffer      = &mut fs::File::create(path).unwrap();

        let camera = &simulation.camera;

        let mut objects: Vec<scene_config::ObjectConfig> = (0..simulation.solid_count()).filter_map(|i| {
            let v = simulation.solid(i);
            let conf = &scene.solids[i];

            if conf.display {
                Some(scene_config::ObjectConfig {
                    path: scene.solids[i].file(&Path::new(&scene.global_config.data_path)).to_str().unwrap().to_string(),
                    scale: conf.scale(),
                    rotation: v.euler_angle(),
                    position: v.final_position(),
                })
            } else {
                None
            }
        }).collect();

        objects.push(scene_config::ObjectConfig {
            path: path.to_str().unwrap().to_string(),
            scale: Vector3::new(1., 1., 1.),
            rotation: Vector3::zeros(),
            position: Vector3::zeros(),
        });

        let config = scene_config::SceneConfig {
            objects: objects,
            params: scene_config::ParamsConfig::default(),
            lights: Vec::new(),
            camera: Camera::new(camera.position(), camera.up(), camera.forward(), 512., 512.),
        };

        mesher.clone().convert_into_obj(&simulation, buffer);
        config.save(&path_yaml).unwrap();

        pb.inc(1);
    });

    pb.finish_with_message("polygonization done");

    Ok(())
}
