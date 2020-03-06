extern crate render;
extern crate sph_common;

use sph_common::Scene;

fn main() {
    let mut sph_scene = Scene::new();
    sph_scene.fill(0.5, 0.4, 0.5);

    println!("{:?}", sph_scene.len());

    let mut scene = render::scene::Scene::new(sph_scene.particle_radius);

    for i in 0..sph_scene.len() {
        scene.push_particle(render::particle::Particle {
            position: sph_scene.particle_dx(i),
            color: (
                0., 0., 1.
            ),
        })
    }

    while scene.render() {
        sph_scene.tick();

        for i in 0..sph_scene.len() {
            scene.get_particle(i).position = sph_scene.particle(i);
        }

        scene.update();
    }
}
