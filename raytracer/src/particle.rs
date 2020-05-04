use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::path::Path;

use flate2::Compression;
use flate2::read::ZlibDecoder;
use flate2::write::ZlibEncoder;
use nalgebra::Vector3;
use serde_derive::*;

#[derive(Deserialize, Serialize)]
pub struct Particles {
    positions: Vec<Vector3<f32>>,
    radii: Vec<f32>,
}

impl Particles {
    pub fn new(positions: Vec<Vector3<f32>>, radii: Vec<f32>) -> Particles {
        assert_eq!(positions.len(), radii.len());
        Particles {
            positions,
            radii,
        }
    }

    pub fn new_cst_radius(positions: Vec<Vector3<f32>>, radius: f32) -> Particles {
        let number_particles = positions.len();
        Particles {
            positions,
            radii: vec![radius; number_particles],
        }
    }

    pub fn len(&self) -> usize { self.positions.len() }

    pub fn get_positions(&self) -> &Vec<Vector3<f32>> { &self.positions }

    pub fn get_radii(&self) -> &Vec<f32> { &self.radii }

    pub fn dump(&self, path: &Path) -> Result<(), std::io::Error> {
        let buffer = BufWriter::new(File::create(path)?);
        let encoder = ZlibEncoder::new(buffer, Compression::default());
        serde_json::to_writer(encoder, self)?;

        Ok(())
    }

    pub fn load(path: &Path) -> Result<Self, std::io::Error> {
        let buffer = BufReader::new(File::open(path)?);
        let decoder = ZlibDecoder::new(buffer);
        let obj: Self = serde_json::from_reader(decoder)?;

        Ok(obj)
    }
}

