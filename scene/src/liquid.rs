use serde_derive::*;
use nalgebra::Vector3;

use sph_common::DFSPH;

#[derive(Debug, Deserialize)]
pub struct LiquidZone
{
    pub from: [f32; 3],
    pub to: [f32; 3],
}

impl LiquidZone
{
    pub fn from(&self) -> Vector3<f32>
    {
        Vector3::new(self.from[0], self.from[1], self.from[2])
    }

    pub fn to(&self) -> Vector3<f32>
    {
        Vector3::new(self.to[0], self.to[1], self.to[2])
    }

    pub fn create_particles(&self, scene: &mut DFSPH)
    {
        let from = self.from();
        let to = self.to();

        let radius = scene.particle_radius() * 2.;
        let step_count = (to - from) / radius;

        let mut count = 0;

        for z in 0..step_count.z as usize {
            let z = z as f32 * radius;
            for y in 0..step_count.y as usize {
                let y = y as f32 * radius;
                for x in 0..step_count.x as usize {
                    let x = x as f32 * radius;
                    scene.add_particle(from.x + x, from.y + y, from.z + z);

                    count += 1;
                }
            }
        }

        println!("{} particles added", count);
    }
}
