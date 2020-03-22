extern crate sph_common;


use std::fs::File;
use std::io::{BufWriter};
use std::time::Instant;

use sph_common::Scene;

fn main() -> std::io::Result<()> {

    let mut sph_scene = Scene::new();
    sph_scene.fill(0.5, 1., 0.5);

    println!("> Number of particles: {}", sph_scene.len());

    let now = Instant::now();
    sph_scene.tick();
    println!("> `Simulation::tick()` elapsed time: {} s", now.elapsed().as_secs_f32());


    let now = Instant::now();
    sph_scene.dump("particles.bin")?;
    println!("> `Simulation::dump()` elapsed time: {} s", now.elapsed().as_secs_f32());

    let now = Instant::now();
    let mut sph_scene = Scene::load("particles.bin")?;
    println!("> `Simulation::load()` elapsed time: {} s", now.elapsed().as_secs_f32());


    let now = Instant::now();
    sph_scene.tick();
    println!("> `Simulation::tick()` elapsed time: {} s", now.elapsed().as_secs_f32());

    Ok(())
}
