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
                        render::event::Key::A => {
                            add_particles(scene_c.add_blocks(&mut sph_scene), &sph_scene, &mut scene);
                        }
                        render::event::Key::Space => {
                            run = !run;
                            let prev_len = sph_scene.len();

                            for i in prev_len..sph_scene.len() {
                                scene.push_particle(render::particle::Particle {
                                    position: sph_scene.particle(i),
                                    color: (0., 0., 1.),
                                });
                            }
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

        // particles position sync in 3d renderin
        for i in 0..sph_scene.len() {
            scene.get_particle(i).position = sph_scene.particle(i);
        }

        total_time += sph_scene.get_time_step();

        scene.debug_text(&format!("\
            dt: {:.6} s\n\
            total: {:.6} s\n\
            nb_particle: {}\n\
            v_max: {:.5} m/s\n\
            fps: {:.3} frame/s\n\
            eye: {}\
        ", sph_scene.get_time_step(), total_time, sph_scene.len(), sph_scene.get_v_max(), 60. / timer.elapsed().as_secs_f32(), scene.camera.eye()));

        // refresh rendering
        scene.update();
    }

    Ok(())
}

