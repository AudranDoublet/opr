use std::fs;
use std::fs::File;
use std::path::{Path, PathBuf};
use std::time::Instant;

use clap::ArgMatches;
use indicatif::{ProgressBar, ProgressStyle};
use kiss3d::camera::camera::Camera;
use nalgebra::Point3;
use sph_common::DFSPH;
use sph_common::mesher::interpolation::InterpolationAlgorithms;
use sph_common::mesher::Mesher;

use rayon::prelude::*;

use crate::simulation::add_particles;
use sph_common::mesher::anisotropication::Anisotropicator;
use sph_common::mesher::types::FluidSnapshotProvider;

fn get_simulation_dumps_paths(folder: &Path) -> Result<Vec<PathBuf>, Box<dyn std::error::Error>> {
    let mut files: Vec<PathBuf> = fs::read_dir(folder)?
        .filter_map(Result::ok)
        .filter_map(|d| d.path().to_str().and_then(|f| if f.ends_with(".sim.bin") { Some(d) } else { None }))
        .map(|d| d.path())
        .collect();

    files.sort();

    Ok(files)
}

fn polygonize(mesher: &mut Mesher, simulation: &impl FluidSnapshotProvider, folder: &Path, idx: usize) -> Result<(), Box<dyn std::error::Error>> {
    let path = folder.join(format!("{:08}.obj", idx));
    let buffer = &mut File::create(path)?;

    mesher.convert_into_obj(simulation, buffer);

    Ok(())
}

fn polygonize_with_gui(mut mesher: Mesher, simulations: Vec<PathBuf>, output_folder: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let mut simulations = simulations.iter();

    let simulation = simulations.next();
    if simulation.is_none() {
        return Ok(());
    }
    let mut simulation = DFSPH::load(&simulation.unwrap()).unwrap();

    let mut renderer = render::scene::Scene::new(simulation.particle_radius());
    renderer.camera.look_at(Point3::new(0.0, 1., -2.), Point3::new(0., 0., 5.)); //FIXME make camera configurable

    add_particles(0..simulation.len(), &simulation, &mut renderer);

    let mut total_time = 0.;
    let mut show_info: bool = true;
    let mut pause: bool = true;
    let mut idx: usize = 0;

    while renderer.render() {
        let timer = Instant::now();

        polygonize(&mut mesher, &simulation, output_folder, idx)?;

        for event in renderer.window.events().iter() {
            match event.value {
                render::event::WindowEvent::Key(key, render::event::Action::Release, _) => {
                    match key {
                        render::event::Key::I => {
                            show_info = !show_info;
                        }
                        render::event::Key::Space => {
                            pause = !pause;
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        if !pause {
            match simulations.next() {
                Some(s) => { simulation = DFSPH::load(&s).unwrap(); }
                None => break
            };

            renderer.clear();
            add_particles(0..simulation.len(), &simulation, &mut renderer);

            total_time += simulation.get_time_step();

            idx += 1;
        }

        if show_info {
            renderer.debug_text(&format!("\
                iteration: {}/{}\n\
                dt: {:.6} s\n\
                total: {:.6} s\n\
                nb_particle: {}\n\
                v_max: {:.5} m/s\n\
                fps: {:.3} frame/s\n\
                eye: {}\
                ", idx, simulations.len() - 1, simulation.get_time_step(), total_time, simulation.len(), simulation.get_v_max(), 1. / timer.elapsed().as_secs_f32(), renderer.camera.eye()));
        }

        renderer.update();
    }

    Ok(())
}


fn polygonize_cli(mesher: Mesher, simulations: Vec<PathBuf>, output_folder: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let pb = ProgressBar::new(simulations.len() as u64);
    pb.set_style(ProgressStyle::default_bar()
        .template("[{elapsed_precise}] [{per_sec}] [{eta_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7}"));
    pb.tick();

    simulations.par_iter().enumerate().for_each(|(i, path)| {
        let dfsph = DFSPH::load(&path).unwrap();

        polygonize(&mut mesher.clone(), &dfsph, output_folder, i).unwrap();
        pb.inc(1);
    });

    pb.finish_with_message("polygonization done");

    Ok(())
}

pub fn main_polygonization(args: &ArgMatches) -> Result<(), Box<dyn std::error::Error>> {
    let output_directory = match args.value_of("output_directory") {
        Some(d) => Path::new(d),
        None => Path::new("./")
    };
    let dump_directory = Path::new(args.value_of("dump_directory").unwrap());
    let disable_interpolation = args.is_present("disable_interpolation");
    let disable_anisotropication = args.is_present("disable_anisotropication");
    let render = args.is_present("render");

    let interpolation_algorithm = match disable_interpolation {
        true => InterpolationAlgorithms::None,
        false => InterpolationAlgorithms::Linear,
    };

    let anisotropicator = if !disable_anisotropication {
        Some(
            Anisotropicator::new(0.9, 15, 40., 20000., 50.)
        )
    } else { None };

    // FIXME: the ISO-VALUE and CUBE-SIZE should be asked in CLI instead of being hardcoded
    let mesher = Mesher::new(0.00001, interpolation_algorithm, anisotropicator);
    // FIXME-END

    fs::create_dir_all(output_directory)?;

    let simulations: Vec<PathBuf> = get_simulation_dumps_paths(dump_directory)?;

    if render {
        polygonize_with_gui(mesher, simulations, output_directory)?;
    } else {
        polygonize_cli(mesher, simulations, output_directory)?;
    }

    Ok(())
}

