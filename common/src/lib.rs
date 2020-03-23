#![feature(test)]

extern crate test;

pub use particle::*;

mod particle;

pub mod constants;
pub mod kernel;
pub mod mesher;

#[bench]
fn bench_sph_tick(b: &mut Bencher) {
    let mut sph_scene = Scene::new();
    sph_scene.fill(0.5, 0.4, 0.5);

    b.iter(|| {
        sph_scene.tick();
    });
}
