use std::path::Path;
use std::time::Instant;

use clap::ArgMatches;
use kiss3d::camera::camera::Camera;
use nalgebra::{Point3, Translation3};
use sph_common::DFSPH;
use sph_scene::Scene;
use std::fs;

use indicatif::{ProgressBar, ProgressStyle};

pub fn add_particles(range: std::ops::Range<usize>, dfsph: &DFSPH, scene: &mut render::scene::Scene) {
    let particles = &dfsph.positions.read().unwrap();

    for i in range {
        let pos = particles[i];

        scene.push_particle(render::particle::Particle {
            visible: true,
            position: (pos.x, pos.y, pos.z),
            color: (0., 0., 1.),
        })
    }
}

fn add_meshes(config: &sph_scene::Scene, scene: &mut render::scene::Scene) {
    for i in 0..config.solids.len() {
        let solid = &config.solids[i];

        if !solid.display {
            continue;
        }

        let data = Path::new(&config.global_config.data_path);
        let mut obj = scene.window.add_obj(&solid.file(data), data, solid.scale());
        obj.set_local_translation(Translation3::new(solid.position[0], solid.position[1], solid.position[2]));
    }
}

fn dump_simulation(simulation: &DFSPH, dump_folder: &Path, idx: usize, verbose: bool) -> Result<(), Box<dyn std::error::Error>> {
    let path = dump_folder.join(format!("{:08}.sim.bin", idx));
    if verbose {
        println!("Dumping scene as `{:?}`", &path);
    }
    let now = Instant::now();
    simulation.dump(&path)?;
    if verbose {
        println!("> `Simulation::dump()` elapsed time: {} s", now.elapsed().as_secs_f32());
    }
    Ok(())
}

fn simulate(scene: Scene, dump_all: bool, dump_folder: &Path, fps: f32) -> Result<(), Box<dyn std::error::Error>> {
    let mut fluid_simulation = scene.load()?;
    let mut total_time = 0.0;
    let mut time_simulated_since_last_frame = fps;
    let mut display_high_speed_only = false;
    let mut pause_on_speed_explosion= false;

    let mut renderer = render::scene::Scene::new(fluid_simulation.particle_radius());
    renderer.camera.look_at(Point3::new(0.0, 1., -2.), Point3::new(0., 0., 5.)); //FIXME make camera configurable

    add_particles(0..fluid_simulation.len(), &fluid_simulation, &mut renderer);
    add_meshes(&scene, &mut renderer);

    let mut show_info: bool = true;
    let mut show_velocity: bool = false;
    let mut pause: bool = true;
    let mut frame_idx: usize = 0;

    while renderer.render() {
        let timer = Instant::now();

        for event in renderer.window.events().iter() {
            match event.value {
                render::event::WindowEvent::Key(key, render::event::Action::Release, _) => {
                    match key {
                        render::event::Key::R => {
                            total_time = 0.0;
                            scene.recreate(&mut fluid_simulation);
                            renderer.clear();
                            add_particles(0..fluid_simulation.len(), &fluid_simulation, &mut renderer);
                        }
                        render::event::Key::A => {
                            add_particles(scene.add_blocks(&mut fluid_simulation), &fluid_simulation, &mut renderer);
                        }
                        render::event::Key::D => {
                            if !dump_all {
                                dump_simulation(&fluid_simulation, dump_folder, frame_idx, true)?;
                            }
                        }
                        render::event::Key::C => {
                            show_velocity = !show_velocity;
                        }
                        render::event::Key::H => {
                            display_high_speed_only = !display_high_speed_only;
                        }
                        render::event::Key::P => {
                            pause_on_speed_explosion = !pause_on_speed_explosion;
                        }
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
            if time_simulated_since_last_frame >= fps {
                if dump_all {
                    dump_simulation(&fluid_simulation, dump_folder, frame_idx, true)?;
                }

                time_simulated_since_last_frame = 0.;
                frame_idx += 1;
            }

            time_simulated_since_last_frame += fluid_simulation.tick();

            let d_v_mean_sq = fluid_simulation.debug_get_v_mean_sq();
            let d_v_max_sq_deviation = fluid_simulation.debug_get_v_max_sq() / d_v_mean_sq;

            // particles position sync in 3d rendering
            let velocities = fluid_simulation.velocities.read().unwrap();
            let positions = fluid_simulation.positions.read().unwrap();

            for i in 0..fluid_simulation.len() {
                let mut particle = renderer.get_particle(i);

                let particle_speed_ratio: f32 = if !d_v_max_sq_deviation.is_nan() {
                    velocities[i].norm_squared() / d_v_max_sq_deviation
                } else {
                    0.
                };

                if pause_on_speed_explosion && particle_speed_ratio > 0.95 {
                    pause = true;
                }

                particle.visible = !display_high_speed_only || particle_speed_ratio > 0.5;
                if show_velocity {
                    particle.color = (particle_speed_ratio.min(1.), 0., (1. - particle_speed_ratio).max(0.));
                } else {
                    particle.color = (0., 0., 1.);
                }

                let pos = positions[i];
                particle.position = (pos.x, pos.y, pos.z);
            }

            total_time += fluid_simulation.get_time_step();

        }

        if show_info {
            renderer.debug_text(&format!("\
                frame: {}\n\
                dt: {:.6} s\n\
                total: {:.6} s\n\
                nb_particle: {}\n\
                v_max: {:.5} m/s\n\
                fps: {:.3} frame/s\n\
                eye: {}\
                ", frame_idx, fluid_simulation.get_time_step(), total_time, fluid_simulation.len(), fluid_simulation.get_v_max(), 1. / timer.elapsed().as_secs_f32(), renderer.camera.eye()));
        }

        renderer.update();
    }

    Ok(())
}

fn simulate_cli(scene: Scene, max_time: f32, dump_folder: &Path, fps: f32) -> Result<(), Box<dyn std::error::Error>> {
    let mut fluid_simulation = scene.load()?;
    let mut total_time = 0.0;
    let mut time_simulated_since_last_frame = fps;

    total_time += fluid_simulation.get_time_step();

    let mut idx = 0;

    let pb = ProgressBar::new(100);

    pb.set_style(ProgressStyle::default_bar()
        .template("[{elapsed}] [{per_sec}] [{eta}] {bar:40.cyan/blue} {pos:>7}/{len:7}"));

    let perc = |x| ((x / max_time) * 100.) as u64;

    while total_time < max_time {
        if time_simulated_since_last_frame >= fps {
            dump_simulation(&fluid_simulation, dump_folder, idx, false)?;
            time_simulated_since_last_frame = 0.;
            idx += 1;
        }

        time_simulated_since_last_frame += fluid_simulation.tick();

        let old = perc(total_time);
        total_time += fluid_simulation.get_time_step();

        let percent = perc(total_time);
        pb.inc(percent - old);
    }

    pb.finish_with_message("polygonization done");

    Ok(())
}

pub fn main_simulation(args: &ArgMatches) -> Result<(), Box<dyn std::error::Error>> {
    let scene_file = args.value_of("scene").unwrap();
    let mut scene_c = sph_scene::load_scene(scene_file)?;

    if args.is_present("nocache") {
        println!("Cache disabled");
        scene_c.global_config.use_cache = false;
    }

    if let Some(data_dir) = args.value_of("data_dir") {
        println!("Custom data directory: {}", data_dir);
        scene_c.global_config.data_path = data_dir.to_string();
    }

    if let Some(cache_dir) = args.value_of("cache_dir") {
        println!("Custom cache directory: {}", cache_dir);
        scene_c.global_config.cache_path = cache_dir.to_string();
    }

    let dump_all = args.is_present("dump_all");
    let dump_folder = match args.value_of("dump_directory") {
        Some(d) => Path::new(d),
        None => Path::new("./")
    };

    fs::create_dir_all(dump_folder)?;

    let fps = 1. / args.value_of("fps").unwrap_or("-1.").parse::<f32>()?;

    let mut max_time = 4.0;
    if let Some(mt) = args.value_of("max_time") {
        max_time = mt.parse()?;
    }

    if args.is_present("no_gui") {
        simulate_cli(scene_c, max_time, dump_folder, fps)
    } else {
        simulate(scene_c, dump_all, dump_folder, fps)
    }
}
