extern crate render;
extern crate sph_common;

use std::time::Instant;

use kiss3d::camera::camera::Camera;
use nalgebra::Point3;
use sph_common::DFSPH;

/*
fn main() {
    let mut sph_scene = DFSPH::new(20.0);
    sph_scene.fill(0.3, 0.5, 0.3);

    println!("{:?}", sph_scene.len());

    for _ in 1..2 {
        sph_scene.tick();
    }
}
*/


fn main() {
    let mut sph_scene = DFSPH::new(20.0);
    sph_scene.fill(0.2, 0.2, 0.2);

    println!("{:?}", sph_scene.len());
    let mut total_time = 0.0;

    let mut scene = render::scene::Scene::new(sph_scene.particle_radius / 2.);
    scene.camera
        .look_at(Point3::new(30., 5., 0.), Point3::new(0., 0., 5.));


    for i in 0..sph_scene.len() {
        scene.push_particle(render::particle::Particle {
            position: sph_scene.particle(i),
            color: (0., 0., 1.),
        })
    }

    while scene.render() {
        let timer = Instant::now();

        // consume events
        for event in scene.window.events().iter() {
            match event.value {
                render::event::WindowEvent::Key(key, render::event::Action::Release, _) => {
                    match key {
                        render::event::Key::R => {
                            scene.clear();
                            sph_scene.clear();
                            // println!("{:?}", scene.camera);
                            // sph_scene.tick();
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
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        // fluid simulation
        sph_scene.tick();

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
}
