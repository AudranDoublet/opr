#![feature(test)]

mod particle;

pub use particle::*;

extern crate test;
use test::Bencher;

#[bench]
fn bench_sph_tick(b: &mut Bencher) {
    let mut sph_scene = Scene::new();
    sph_scene.fill(0.5, 0.4, 0.5);

    b.iter(|| {
        sph_scene.tick();
    });
}
