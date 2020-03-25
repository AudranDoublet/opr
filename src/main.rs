extern crate render;
extern crate sph_common;

use std::fs::File;
use std::time::Instant;

use nalgebra::Point3;
use sph_common::Scene;
use sph_common::mesher::interpolation::InterpolationAlgorithms;
use sph_common::mesher::Mesher;

fn main() -> Result<(), std::io::Error> {
    let mesher = Mesher::new(2., 0.5, InterpolationAlgorithms::None);

    let mut sph_scene = Scene::new();
    sph_scene.fill(0.5, 0.4, 0.5);

    println!("{:?}", sph_scene.len());

    let mut scene = render::scene::Scene::new(sph_scene.particle_radius / 3.);
    scene
        .camera
        .look_at(Point3::new(0.0, 20.0, -50.0), Point3::new(10.0, 10.0, 0.0));

    for i in 0..sph_scene.len() {
        scene.push_particle(render::particle::Particle {
            position: sph_scene.particle(i),
            color: (0., 0., 1.),
        })
    }

    let mut pause = false;

    let mut iter_idx: usize = 0;
    while scene.render() {
        // consume events
        for event in scene.window.events().iter() {
            match event.value {
                render::event::WindowEvent::Key(key, render::event::Action::Release, _) => {
                    match key {
                        render::event::Key::R => {
                            println!("audran tg");
                            scene.clear();
                            sph_scene.clear();
                        }
                        render::event::Key::Space => {
                            let prev_len = sph_scene.len();
                            sph_scene.fill_part(0.4, 0.6, 0.4, 0.2, 0.4, 0.2);

                            println!("{:?}", sph_scene.len());

                            for i in prev_len..sph_scene.len() {
                                scene.push_particle(render::particle::Particle {
                                    position: sph_scene.particle(i),
                                    color: (0., 0., 1.),
                                });
                            }
                        }
                        render::event::Key::D => {
                            let path = &format!("{:05}.bin", iter_idx);
                            println!("Dumping scene as `{}`", path);
                            let now = Instant::now();
                            sph_scene.dump(path)?;
                            println!("> `Simulation::dump()` elapsed time: {} s", now.elapsed().as_secs_f32());
                        }
                        render::event::Key::A => {
                            let path = &format!("{:08}.obj", iter_idx);
                            println!("Meshing scene as `{}`", path);
                            let buffer = &mut File::create(path)?;
                            let now = Instant::now();
                            mesher.to_obj(&sph_scene, buffer);
                            println!("> `Simulation::meshing()` elapsed time: {} s", now.elapsed().as_secs_f32());
                        }
                        render::event::Key::P => {
                            pause = !pause;
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        let path = &format!("{:08}.obj", iter_idx);
        println!("Meshing scene as `{}`", path);
        let buffer = &mut File::create(path)?;
        let now = Instant::now();
        mesher.to_obj(&sph_scene, buffer);
        println!("> `Simulation::meshing()` elapsed time: {} s", now.elapsed().as_secs_f32());

        // fluid simulation
        if !pause {
            let now = Instant::now();
            sph_scene.tick();
            println!("> `Simulation::tick()` elapsed time: {} s", now.elapsed().as_secs_f32());
        }

        // particles position sync in 3d rendering
        for i in 0..sph_scene.len() {
            scene.get_particle(i).position = sph_scene.particle(i);
        }

        // refresh rendering
        if !pause {
            scene.update();
        }

        iter_idx += 1;
    }

    Ok(())
}
