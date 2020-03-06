extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::camera::FirstPerson;
use kiss3d::light::Light;
use kiss3d::window::Window;
use na::{Translation3, Vector3};

use self::na::{Point3, Quaternion, UnitQuaternion};
use crate::particle::Particle;

pub struct Scene {
    window: Window,
    particles: std::vec::Vec<Particle>,
    particle_radius: f32,
    nodes: std::vec::Vec<kiss3d::scene::SceneNode>,
    pub camera: FirstPerson,
}

impl Scene {
    pub fn new(particle_radius: f32) -> Scene {
        let mut scene = Scene {
            window: Window::new("Audran is stupid"),
            camera: FirstPerson::new(Point3::origin(), Point3::origin()),
            particles: vec![],
            particle_radius,
            nodes: vec![],
        };

        scene.window.set_light(Light::StickToCamera);

        scene
    }

    pub fn push_particle(&mut self, p: Particle) {
        self.particles.push(p);
        self.nodes
            .push(self.window.add_sphere(self.particle_radius));
        self._sync_particle(self.particles.len() - 1);
    }

    pub fn get_particle(&mut self, idx: usize) -> &mut Particle {
        self.particles.get_mut(idx).unwrap()
    }

    fn _sync_particle(&mut self, idx: usize) {
        let node = &mut self.nodes[idx];
        let particle = &self.particles[idx];

        let (x, y, z) = particle.position;
        let (r, g, b) = particle.color;

        node.set_local_translation(Translation3::new(x, y, z));
        node.set_color(r, g, b);
    }

    pub fn update(&mut self) {
        for i in 0..self.nodes.len() {
            self._sync_particle(i);
        }
    }

    pub fn render(&mut self) -> bool {
        self.window.render_with_camera(&mut self.camera)
    }
}
