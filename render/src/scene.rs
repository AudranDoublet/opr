extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::camera::FirstPerson;
use kiss3d::light::Light;
use kiss3d::text::Font;
use kiss3d::window::Window;

use crate::particle::{Particle, V3};

use self::na::{Point2, Point3, Translation3};
use kiss3d::scene::SceneNode;

pub struct Scene {
    pub window: Window,
    particles: std::vec::Vec<Particle>,
    particle_radius: f32,
    fluid_nodes: std::vec::Vec<kiss3d::scene::SceneNode>,
    pub diffuse: kiss3d::scene::SceneNode,
    pub camera: FirstPerson,
}

impl Scene {
    pub fn new(particle_radius: f32) -> Scene {
        let window= Window::new("OPR - Fluid Simulation");
        let mut scene = Scene {
            window,
            camera: FirstPerson::new(Point3::origin(), Point3::origin()),
            particles: vec![],
            particle_radius,
            fluid_nodes: vec![],
            diffuse: SceneNode::new_empty(),
        };

        scene.window.set_light(Light::StickToCamera);

        scene
    }

    pub fn clear(&mut self) {
        for node in &mut self.fluid_nodes {
            self.window.remove_node(node);
        }

        self.window.remove_node(&mut self.diffuse);

        self.fluid_nodes.clear();
        self.particles.clear();
    }

    pub fn get_particle_radius(&self) -> f32 {
        self.particle_radius
    }

    pub fn push_particle(&mut self, p: Particle) {
        self.particles.push(p);
        self.fluid_nodes
            .push(self.window.add_sphere(self.particle_radius));
        self._sync_particle(self.particles.len() - 1);
    }

    pub fn get_particle(&mut self, idx: usize) -> &mut Particle {
        self.particles.get_mut(idx).unwrap()
    }

    fn _sync_particle(&mut self, idx: usize) {
        let node = &mut self.fluid_nodes[idx];
        let particle = &self.particles[idx];

        let (x, y, z) = particle.position;
        let (r, g, b) = particle.color;

        node.set_local_translation(Translation3::new(x, y, z));
        node.set_color(r, g, b);
        node.set_visible(particle.visible);
    }

    pub fn update(&mut self) {
        for i in 0..self.fluid_nodes.len() {
            self._sync_particle(i);
        }
    }

    pub fn debug_text(&mut self, text: &str) {
        self.window.draw_text(text, &Point2::new(10., 0.), 30.0, &Font::default(), &Point3::new(1., 1., 1.))
    }

    pub fn render(&mut self) -> bool {
        self.window.render_with_camera(&mut self.camera)
    }
}
