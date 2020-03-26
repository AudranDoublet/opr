#![feature(clamp)]

extern crate render;
extern crate sph_common;
extern crate sph_scene;

#[macro_use]
extern crate clap;

//mod extract_grid;

use clap::App;

//use std::path::Path;
//use sph_common::{DFSPH, DiscreteGrid, mesh::Mesh, kernels::CubicSpine};
//use nalgebra::{Point3, Vector3};

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

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let matches = App::from_yaml(load_yaml!("cli.yml")).get_matches();

    let nocache = matches.value_of("nocache").unwrap_or(false);
    let data_dir = matches.value_of("data_dir").unwrap_or("data/");
    let cache_dir = matches.value_of("cache_dir").unwrap_or("cache/");

    Ok(())
}

/*
fn main() {
    let mut sph_scene = DFSPH::new(20.0);
    sph_scene.fill(0.01, 0.01, 0.01);

    println!("{:?}", sph_scene.len());

    let mut scene = render::scene::Scene::new(sph_scene.particle_radius);
    scene
        .camera
        .look_at(Point3::new(10.0, 0., -1.0), Point3::new(0.0, 0.0, 0.));

    for i in 0..sph_scene.len() {
        scene.push_particle(render::particle::Particle {
            position: sph_scene.particle(i),
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
                            //scene.clear();
                            //sph_scene.clear();
                            sph_scene.tick();
                        },
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
*/
