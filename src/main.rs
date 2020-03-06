extern crate render;
extern crate sph_common;

use nalgebra::{Point3, Vector, Vector3};
use sph_common::Scene;

fn main() {
    let mut sph_scene = Scene::new();
    sph_scene.fill(0.5, 0.4, 0.5);

    println!("{:?}", sph_scene.len());

    let mut scene = render::scene::Scene::new(sph_scene.particle_radius / 3.);
    scene
        .camera
        .look_at(Point3::new(0.0, 20.0, -50.0), Point3::new(10.0, 10.0, 0.0));

    for i in 0..sph_scene.len() {
        scene.push_particle(render::particle::Particle {
            position: sph_scene.particle_dx(i),
            color: (0., 0., 1.),
        })
    }

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
                        },
                        render::event::Key::Space => {
                            let prev_len = sph_scene.len();
                            sph_scene.fill_part(0.4, 0.6, 0.4, 0.2, 0.4, 0.2);

                            println!("{:?}", sph_scene.len());

                            for i in prev_len..sph_scene.len() {
                                scene.push_particle(render::particle::Particle {
                                    position: sph_scene.particle_dx(i),
                                    color: (0., 0., 1.),
                                });
                            }
                        },
                        _ => {}
                    }
                },
            _ => {}
            }

        }


        // fluid simulation
        sph_scene.tick();

        // particles position sync in 3d renderin
        for i in 0..sph_scene.len() {
            scene.get_particle(i).position = sph_scene.particle(i);
        }

        // refresh rendering
        scene.update();
    }
}
