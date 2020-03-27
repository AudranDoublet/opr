pub type V3 = (f32, f32, f32);

pub struct Particle {
    pub position: V3,
    pub color: V3,
    pub visible: bool,
}

impl Default for Particle {
    fn default() -> Particle {
        Particle {
            position: (0.0, 0.0, 0.0),
            color: (0.0, 0.0, 0.0),
            visible: true,
        }
    }
}
