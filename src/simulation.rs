extern crate pipeline;

use std::fs;
use std::path::Path;
use std::time::Instant;

use bubbler::bubbler::Bubbler;
use bubbler::diffuse_particle::DiffuseParticleType;
use clap::ArgMatches;
use kiss3d::{camera::camera::Camera, scene::SceneNode};
use nalgebra::{Point3, Translation3};
use sph_common::Simulation;
use sph_scene::Scene;

pub fn add_particles(range: std::ops::Range<usize>, dfsph: &Simulation, scene: &mut render::scene::Scene) {
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

fn add_meshes(dfsph: &Simulation, config: &sph_scene::Scene, scene: &mut render::scene::Scene) -> Vec<Option<SceneNode>> {
    let mut result = Vec::new();

    for i in 0..config.solids.len() {
        let solid = &config.solids[i];
        let center = dfsph.solid(i).center_of_mass.component_div(&solid.scale());
        let position = dfsph.solid(i).position();

        if !solid.display {
            result.push(None);
        } else {
            let data = Path::new(&config.global_config.data_path);
            let path = solid.file(data);
            let mut obj = scene.window.add_obj(&path, &path.parent().unwrap(), solid.scale());

            obj.modify_vertices(&mut |v| {
                v.iter_mut().for_each(|t| *t -= center);
            });

            obj.set_local_translation(Translation3::new(position[0], position[1], position[2]));

            result.push(Some(obj));
        }
    }

    result
}

fn dump_simulation(simulation: &Simulation, dump_folder: &Path, idx: usize, verbose: bool) -> Result<(), Box<dyn std::error::Error>> {
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

fn update_diffuse(renderer: &mut render::scene::Scene, bubbler: &Bubbler) {
    renderer.window.remove_node(&mut renderer.diffuse);
    renderer.diffuse = renderer.window.add_group();

    let radius = renderer.get_particle_radius();

    for diffuse in bubbler.get_particles() {
        let diffuse_radius = match diffuse.kind {
            DiffuseParticleType::Spray => radius * 0.4,
            DiffuseParticleType::Bubble => radius * 0.8,
            DiffuseParticleType::Foam => radius * 0.7,
        };
        let (r, g, b) = match diffuse.kind {
            DiffuseParticleType::Spray => (1., 1., 1.),
            DiffuseParticleType::Bubble => (1., 0., 1.),
            DiffuseParticleType::Foam => (1., 1., 0.),
        };

        let mut sphere = renderer.diffuse.add_sphere(diffuse_radius);
        let (x, y, z) = (diffuse.position.x, diffuse.position.y, diffuse.position.z);
        sphere.set_local_translation(Translation3::new(x, y, z));
        sphere.set_color(r, g, b);
    }
}

fn simulate(scene: &Scene, dump_all: bool, dump_folder: &Path, fps: f32) -> Result<(), Box<dyn std::error::Error>> {
    let mut fluid_simulation = scene.load()?;
    let mut bubbler = Bubbler::new(scene.bubbler_config);
    let mut total_time = 0.0;
    let mut time_simulated_since_last_frame = fps;
    let mut display_high_speed_only = false;
    let mut pause_on_speed_explosion = false;

    let mut collision_size = 5.;

    let mut renderer = render::scene::Scene::new(fluid_simulation.particle_radius());
    renderer.camera.look_at(Point3::new(0.0, 1., -2.), Point3::new(0., 0., 5.)); //FIXME make camera configurable
    renderer.window.set_point_size(collision_size);

    add_particles(0..fluid_simulation.len(), &fluid_simulation, &mut renderer);
    fluid_simulation.init_forces();

    let mut meshes = add_meshes(&fluid_simulation, &scene, &mut renderer);

    let mut show_info: bool = true;
    let mut show_velocity: bool = false;
    let mut show_collisions: bool = true;
    let mut hide_solids = false;
    let mut pause: bool = true;
    let mut frame_idx: usize = 0;

    while renderer.render() {
        let timer = Instant::now();

        for event in renderer.window.events().iter() {
            match event.value {
                render::event::WindowEvent::Key(key, render::event::Action::Release, _) => match key {
                    render::event::Key::R => {
                        total_time = 0.0;
                        scene.recreate(&mut fluid_simulation)?;
                        renderer.clear();
                        add_particles(0..fluid_simulation.len(), &fluid_simulation, &mut renderer);
                        fluid_simulation.init_forces();
                    }
                    render::event::Key::A => {
                        add_particles(scene.add_blocks(&mut fluid_simulation)?, &fluid_simulation, &mut renderer);
                    }
                    render::event::Key::D => {
                        if !dump_all {
                            dump_simulation(&fluid_simulation, dump_folder, frame_idx, true)?;
                        }
                    }
                    render::event::Key::Y => {
                        show_collisions = !show_collisions;
                    }
                    render::event::Key::H => {
                        hide_solids = !hide_solids;
                        for i in 0..fluid_simulation.solid_count() {
                            if let Some(mesh) = &mut meshes[i] {
                                mesh.set_visible(!hide_solids);
                            }
                        }
                    }
                    render::event::Key::C => {
                        show_velocity = !show_velocity;
                    }
                    render::event::Key::S => {
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
                render::event::WindowEvent::Key(key, render::event::Action::Press, render::event::Modifiers::Shift) => match key {
                    render::event::Key::PageDown => {
                        collision_size = (collision_size - 0.25).max(1.);
                        renderer.window.set_point_size(collision_size);
                    }
                    render::event::Key::PageUp => {
                        collision_size = (collision_size + 0.25).min(10.);
                        renderer.window.set_point_size(collision_size);
                    }
                    _ => {}
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

            let prev = fluid_simulation.len();
            time_simulated_since_last_frame += fluid_simulation.tick();
            add_particles(prev..fluid_simulation.len(), &fluid_simulation, &mut renderer);
            /*
            if bubbler.tick(&fluid_simulation) {
                update_diffuse(&mut renderer, &bubbler);
            }
            */

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


            if !hide_solids {
                for i in 0..fluid_simulation.solid_count() {
                    let solid = fluid_simulation.solid(i);

                    if let Some(mesh) = &mut meshes[i] {
                        mesh.set_local_rotation(solid.rotation());
                        mesh.set_local_translation(Translation3::from(solid.center_position()));
                    }
                }
            }

            total_time += fluid_simulation.get_time_step();
        }

        if show_collisions {
            fluid_simulation.debug_get_solid_collisions().iter().for_each(|v| {
                renderer.window.draw_point(&Point3::from(*v), &Point3::new(0., 0., 1.));
            });
        }

        if show_info {
            renderer.debug_text(&format!("\
                frame: {}\n\
                dt: {:.6} s\n\
                total: {:.6} s\n\
                nb_particle: {}\n\
                v_max: {:.5} m/s\n\
                show_solids: {:}\n\
                show_collisions: {:}\n\
                collision_size: {:.3}\n\
                nb_solid_collisions: {}\n\
                fps: {:.3} frame/s\n\
                eye: {}\
                ", frame_idx, fluid_simulation.get_time_step(), total_time, fluid_simulation.len(), fluid_simulation.get_v_max(),
                                         !hide_solids,
                                         show_collisions,
                                         collision_size,
                                         fluid_simulation.debug_get_solid_collisions().len(),
                                         1. / timer.elapsed().as_secs_f32(),
                                         renderer.camera.eye()));
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

    scene_c.simulation_config.fps = args.value_of("fps").unwrap_or("-1.").parse::<f32>()?;
    scene_c.simulation_config.max_time = args.value_of("max_time").unwrap_or("4.0").parse::<f32>()?;

    if args.is_present("no_gui") {
        pipeline::simulate::pipeline_simulate(&scene_c, dump_folder)
    } else {
        simulate(&scene_c, dump_all, dump_folder, 1. / scene_c.simulation_config.fps)
    }
}
