use std::fs;

use indicatif::{ProgressBar, ProgressStyle};
use std::path::Path;

use sph_common::DFSPH;
use sph_scene::Scene;

fn dump_simulation(simulation: &DFSPH, dump_folder: &Path, idx: usize) -> Result<(), Box<dyn std::error::Error>> {
    let path = dump_folder.join(format!("{:08}.sim.bin", idx));
    simulation.dump(&path)?;

    Ok(())
}

pub fn pipeline_simulate(scene: &Scene, dump_folder: &Path) -> Result<(), Box<dyn std::error::Error>> {
    fs::create_dir_all(dump_folder)?;

    let max_time = scene.simulation_config.max_time;
    let fps = 1. / scene.simulation_config.fps;

    let mut fluid_simulation = scene.load()?;
    let mut total_time = 0.0;
    let mut time_simulated_since_last_frame = fps;

    total_time += fluid_simulation.get_time_step();

    let mut idx = 0;

    let pb = ProgressBar::new(100);
    pb.set_style(ProgressStyle::default_bar() .template("[{elapsed}] [{per_sec}] [{eta}] {bar:40.cyan/blue} {pos:>7}/{len:7}"));

    let perc = |x| ((x / max_time) * 100.) as u64;

    while total_time < max_time {
        if time_simulated_since_last_frame >= fps {
            dump_simulation(&fluid_simulation, dump_folder, idx)?;
            time_simulated_since_last_frame = 0.;
            idx += 1;
        }

        time_simulated_since_last_frame += fluid_simulation.tick();

        let old = perc(total_time);
        total_time += fluid_simulation.get_time_step();

        let percent = perc(total_time);
        pb.inc(percent - old);
    }

    pb.finish_with_message("simulation done");

    Ok(())
}