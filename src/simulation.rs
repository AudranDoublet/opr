use std::path::Path;
use std::time::Instant;

use clap::ArgMatches;
use kiss3d::camera::camera::Camera;
use nalgebra::Point3;
use sph_common::DFSPH;
use sph_scene::Scene;
use std::fs;

pub fn add_particles(range: std::ops::Range<usize>, dfsph: &DFSPH, scene: &mut render::scene::Scene) {
    for i in range {
        scene.push_particle(render::particle::Particle {
            position: dfsph.particle(i),
            color: (0., 0., 1.),
        })
    }
}

fn dump_simulation(simulation: &DFSPH, dump_folder: &Path, idx: usize) -> Result<(), Box<dyn std::error::Error>> {
    let path = dump_folder.join(format!("{:08}.sim.bin", idx));
    println!("Dumping scene as `{:?}`", &path);
    let now = Instant::now();
    simulation.dump(&path)?;
    println!("> `Simulation::dump()` elapsed time: {} s", now.elapsed().as_secs_f32());
    Ok(())
}

fn simulate(scene: Scene, dump_all: bool, dump_folder: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let mut fluid_simulation = scene.load()?;
    let mut total_time = 0.0;

    let mut renderer = render::scene::Scene::new(fluid_simulation.particle_radius());
    renderer.camera.look_at(Point3::new(0.0, 1., -2.), Point3::new(0., 0., 5.)); //FIXME make camera configurable

    add_particles(0..fluid_simulation.len(), &fluid_simulation, &mut renderer);

    let mut show_info: bool = true;
    let mut pause: bool = true;
    let mut idx: usize = 0;

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
                                dump_simulation(&fluid_simulation, dump_folder, idx)?;
                            }
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
            if dump_all {
                dump_simulation(&fluid_simulation, dump_folder, idx)?;
            }

            fluid_simulation.tick();

            for i in 0..fluid_simulation.len() {
                renderer.get_particle(i).position = fluid_simulation.particle(i);
            }

            total_time += fluid_simulation.get_time_step();

            idx += 1;
        }

        if show_info {
            renderer.debug_text(&format!("\
                iteration: {}\n\
                dt: {:.6} s\n\
                total: {:.6} s\n\
                nb_particle: {}\n\
                v_max: {:.5} m/s\n\
                fps: {:.3} frame/s\n\
                eye: {}\
                ", idx, fluid_simulation.get_time_step(), total_time, fluid_simulation.len(), fluid_simulation.get_v_max(), 1. / timer.elapsed().as_secs_f32(), renderer.camera.eye()));
        }

        renderer.update();
    }

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

    simulate(scene_c, dump_all, dump_folder)
}

