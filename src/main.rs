#[macro_use]
extern crate clap;
extern crate render;
extern crate sph_scene;

use std::time::Instant;
use std::path::Path;

use clap::App;
use kiss3d::camera::camera::Camera;
use nalgebra::{Point3, Translation3};
use sph_common::DFSPH;

fn add_particles(range: std::ops::Range<usize>, dfsph: &DFSPH, scene: &mut render::scene::Scene) {
    for i in range {
        scene.push_particle(render::particle::Particle {
            position: dfsph.particle(i),
            color: (0., 0., 1.),
            visible: true
        })
    }
}

fn add_meshes(dfsph: &DFSPH, config: &sph_scene::Scene, scene: &mut render::scene::Scene) {
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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let conf = load_yaml!("cli.yml");
    let matches = App::from_yaml(conf).get_matches();

    let scene_file = matches.value_of("SCENE").unwrap();
    let mut scene_c = sph_scene::load_scene(scene_file)?;

    //scene.window
    if matches.is_present("nocache") {
        println!("Cache disabled");
        scene_c.global_config.use_cache = false;
    }

    if let Some(data_dir) = matches.value_of("data_dir") {
        println!("Custom data directory: {}", data_dir);
        scene_c.global_config.data_path = data_dir.to_string();
    }

    if let Some(cache_dir) = matches.value_of("cache_dir") {
        println!("Custom cache directory: {}", cache_dir);
        scene_c.global_config.cache_path = cache_dir.to_string();
    }

    let mut sph_scene = scene_c.load()?;
    let mut total_time = 0.0;
    let mut display_high_speed_only = false;
    let mut pause_on_speed_explosion= false;

    let mut scene = render::scene::Scene::new(sph_scene.particle_radius());
    scene.camera.look_at(Point3::new(0.0, 1., -2.), Point3::new(0., 0., 5.)); //FIXME make camera configurable

    add_particles(0..sph_scene.len(), &sph_scene, &mut scene);
    add_meshes(&sph_scene, &scene_c, &mut scene);

    let mut run: bool = false;

    while scene.render() {
        let timer = Instant::now();

        // consume events
        for event in scene.window.events().iter() {
            match event.value {
                render::event::WindowEvent::Key(key, render::event::Action::Release, _) => {
                    match key {
                        render::event::Key::R => {
                            total_time = 0.0;

                            scene_c.recreate(&mut sph_scene);
                            scene.clear();
                            add_particles(0..sph_scene.len(), &sph_scene, &mut scene);
                        }
                        render::event::Key::H => {
                            display_high_speed_only = !display_high_speed_only;
                        }
                        render::event::Key::P => {
                            pause_on_speed_explosion = !pause_on_speed_explosion;
                        }
                        render::event::Key::A => {
                            add_particles(scene_c.add_blocks(&mut sph_scene), &sph_scene, &mut scene);
                        }
                        render::event::Key::Space => {
                            run = !run;
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        // fluid simulation
        if run {
            sph_scene.tick();
        }

        let v_max = sph_scene.get_v_max();

        // particles position sync in 3d rendering
        for i in 0..sph_scene.len() {
            let mut particle = scene.get_particle(i);
            let particle_speed_ratio = if v_max > 0.00001 {
                sph_scene.velocity(i).norm() / v_max
            } else {
                0.
            };

            if pause_on_speed_explosion && particle_speed_ratio > 0.95 {
                run = false;
            }

            particle.visible = !display_high_speed_only || particle_speed_ratio > 0.5;
            particle.color = (particle_speed_ratio.min(1.), 0., (1. - particle_speed_ratio).max(0.));
            particle.position = sph_scene.particle(i);
        }

        total_time += sph_scene.get_time_step();

        scene.debug_text(&format!("\
            pause_on_speed_explosion: {} \n\
            show_only_high_velocity: {} \n\
            dt: {:.6} s\n\
            total: {:.6} s\n\
            nb_particle: {}\n\
            v_max: {:.5} m/s\n\
            fps: {:.3} frame/s\n\
            eye: {}\
        ", pause_on_speed_explosion, display_high_speed_only, sph_scene.get_time_step(), total_time, sph_scene.len(), sph_scene.get_v_max(), 60. / timer.elapsed().as_secs_f32(), scene.camera.eye()));

        // refresh rendering
        scene.update();
    }

    Ok(())
}

