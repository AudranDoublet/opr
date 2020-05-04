use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::path::Path;

use flate2::Compression;
use flate2::read::ZlibDecoder;
use flate2::write::ZlibEncoder;
use sph::Simulation;

pub fn dump(path: &Path, simulation: &Simulation) -> Result<(), std::io::Error> {
    let buffer = BufWriter::new(File::create(path)?);
    let encoder = ZlibEncoder::new(buffer, Compression::default());
    serde_json::to_writer(encoder, &simulation)?;

    Ok(())
}

pub fn load(path: &Path) -> Result<Simulation, std::io::Error> {
    let buffer = BufReader::new(File::open(path)?);
    let decoder = ZlibDecoder::new(buffer);
    let mut simulation: Simulation = serde_json::from_reader(decoder)?;

    simulation.sync();

    Ok(simulation)
}
