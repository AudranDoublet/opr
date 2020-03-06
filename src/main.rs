extern crate nalgebra;
extern crate rand;
extern crate render;

use nalgebra::{min, DimNameSub, RowVector3, Vector3};
use rand::distributions::{Distribution, Uniform};
use rand::prelude::ThreadRng;
use rand::Rng;

use render::particle::Particle;

struct FakeBoundary {
    v_min: Vector3<f32>,
    v_len: Vector3<f32>,
    rng: ThreadRng,
    distribution: Uniform<f32>,
}

impl FakeBoundary {
    pub fn new(A: Vector3<f32>, B: Vector3<f32>) -> FakeBoundary {
        FakeBoundary {
            v_min: Vector3::new(A.x.min(B.x), A.y.min(B.y), A.z.min(B.z)),
            v_len: (A - B).abs(),
            rng: rand::thread_rng(),
            distribution: Uniform::from(0.0..1.0),
        }
    }

    fn rnd_vec3(&mut self) -> Vector3<f32> {
        Vector3::new(
            self.distribution.sample(&mut self.rng),
            self.distribution.sample(&mut self.rng),
            self.distribution.sample(&mut self.rng),
        )
    }

    pub fn generate_rnd_point(&mut self) -> Vector3<f32> {
        self.rnd_vec3().component_mul(&self.v_len) + &self.v_min
    }
}

fn main() {
    let mut boundaries = FakeBoundary::new(
        Vector3::new(-50.0, -50.0, -50.0),
        Vector3::new(50.0, 50.0, 50.0),
    );

    let particle_nb = 2000;
    let particle_radius: f32 = 0.5;

    let mut scene = render::scene::Scene::new(particle_radius);

    for _ in 0..particle_nb {
        let p = boundaries.generate_rnd_point();

        scene.push_particle(Particle {
            position: (p.x, p.y, p.z),
            color: (
                boundaries.distribution.sample(&mut boundaries.rng),
                boundaries.distribution.sample(&mut boundaries.rng),
                boundaries.distribution.sample(&mut boundaries.rng),
            ),
        })
    }

    while scene.render() {
        scene.update();
    }
}
