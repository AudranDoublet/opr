use std::path::{Path, PathBuf};
use std::fs;

use indicatif::{ProgressBar, ProgressStyle};

use sph_scene::Scene;

use sph_common::DFSPH;
use sph_common::mesher::interpolation::InterpolationAlgorithms;
use sph_common::mesher::Mesher;
use sph_common::mesher::anisotropication::Anisotropicator;

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
        let simulation  = DFSPH::load(&path).unwrap();
        let path        = dump_directory.join(format!("{:08}.obj", idx));
        let buffer      = &mut fs::File::create(path).unwrap();

        mesher.clone().to_obj(&simulation, buffer);

        pb.inc(1);
    });

    pb.finish_with_message("polygonization done");

    Ok(())
}
