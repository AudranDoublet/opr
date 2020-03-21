#![feature(test)]

extern crate test;

use test::Bencher;

pub use particle::*;

mod particle;

#[bench]
fn bench_sph_tick(b: &mut Bencher) {
    let mut sph_scene = Scene::new();
    sph_scene.fill(0.5, 0.4, 0.5);

    b.iter(|| {
        sph_scene.tick();
    });
}
