extern crate render;
extern crate sph_common;

use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

use nalgebra::{Point3, Vector3};

fn main() -> Result<(), std::io::Error> {
    let scale: Vector3<f32> = Vector3::new(0.5, 0.5, 0.5);


    let mut scene = render::scene::Scene::new(3.);
    scene
        .camera
        .look_at(Point3::new(0.0, 20.0, -50.0), Point3::new(10.0, 10.0, 0.0));

    let sample_dir = Path::new("./");
    let mut files: Vec<PathBuf> = fs::read_dir(sample_dir)?
        .filter_map(Result::ok)
        .filter(|d| if let Some(e) = d.path().extension() { e == "obj" } else { false })
        .map(|d| d.path())
        .collect();

    files.sort();

    if files.len() == 0 {
        panic!("no obj");
    }

    let mut i_obj: usize = 0;

    // Teapot
    let mut obj = scene.window.add_obj(&files[i_obj], &sample_dir, scale);
    obj.enable_backface_culling(false);

    let mut pause = true;
    let mut now = Instant::now();

    let mut time_step = 0.;

    // consume events
    while scene.render() {
        if !pause && now.elapsed().as_secs_f32() > time_step {
            i_obj = (i_obj + 1) % files.len();
            scene.window.remove_node(&mut obj);
            obj = scene.window.add_obj(&files[i_obj], &sample_dir, scale);
            obj.enable_backface_culling(false);
            // obj.recompute_normals();
            // scene.render();
            now = Instant::now();
        }

        for event in scene.window.events().iter() {

            match event.value {
                render::event::WindowEvent::Key(key, render::event::Action::Release, _) => {
                    match key {
                        render::event::Key::P => {
                            pause = !pause;
                        }
                        render::event::Key::PageUp => {
                            time_step = (time_step + 0.1).min(3.);
                        }
                        render::event::Key::PageDown => {
                            time_step = (time_step - 0.1).max(0.);
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }
    }

    Ok(())
}
// fn main() -> Result<(), std::io::Error> {
//     let mesher = Mesher::new(0.3, 0.1);
//
//     let mut sph_scene = Scene::new();
//     sph_scene.fill(0.5, 0.4, 0.5);
//
//     println!("{:?}", sph_scene.len());
//
//     let mut scene = render::scene::Scene::new(sph_scene.particle_radius / 3.);
//     scene
//         .camera
//         .look_at(Point3::new(0.0, 20.0, -50.0), Point3::new(10.0, 10.0, 0.0));
//
//     for i in 0..sph_scene.len() {
//         scene.push_particle(render::particle::Particle {
//             position: sph_scene.particle(i),
//             color: (0., 0., 1.),
//         })
//     }
//
//     let mut pause = false;
//
//     let mut iter_idx: usize = 0;
//     while scene.render() {
//         // consume events
//         for event in scene.window.events().iter() {
//             match event.value {
//                 render::event::WindowEvent::Key(key, render::event::Action::Release, _) => {
//                     match key {
//                         render::event::Key::R => {
//                             println!("audran tg");
//                             scene.clear();
//                             sph_scene.clear();
//                         }
//                         render::event::Key::Space => {
//                             let prev_len = sph_scene.len();
//                             sph_scene.fill_part(0.4, 0.6, 0.4, 0.2, 0.4, 0.2);
//
//                             println!("{:?}", sph_scene.len());
//
//                             for i in prev_len..sph_scene.len() {
//                                 scene.push_particle(render::particle::Particle {
//                                     position: sph_scene.particle(i),
//                                     color: (0., 0., 1.),
//                                 });
//                             }
//                         }
//                         render::event::Key::D => {
//                             let path = &format!("{:05}.bin", iter_idx);
//                             println!("Dumping scene as `{}`", path);
//                             let now = Instant::now();
//                             sph_scene.dump(path)?;
//                             println!("> `Simulation::dump()` elapsed time: {} s", now.elapsed().as_secs_f32());
//                         }
//                         render::event::Key::A => {
//                             let path = &format!("{:08}.obj", iter_idx);
//                             println!("Meshing scene as `{}`", path);
//                             let buffer = &mut File::create(path)?;
//                             let now = Instant::now();
//                             mesher.to_obj(&sph_scene, buffer);
//                             println!("> `Simulation::meshing()` elapsed time: {} s", now.elapsed().as_secs_f32());
//                         }
//                         render::event::Key::P => {
//                             pause = !pause;
//                         }
//                         _ => {}
//                     }
//                 }
//                 _ => {}
//             }
//         }
//
//         // fluid simulation
//         if !pause {
//             let now = Instant::now();
//             sph_scene.tick();
//             println!("> `Simulation::tick()` elapsed time: {} s", now.elapsed().as_secs_f32());
//         }
//
//         // particles position sync in 3d rendering
//         for i in 0..sph_scene.len() {
//             scene.get_particle(i).position = sph_scene.particle(i);
//         }
//
//         // refresh rendering
//         if !pause {
//             scene.update();
//         }
//
//         iter_idx += 1;
//     }
//
//     Ok(())
// }
