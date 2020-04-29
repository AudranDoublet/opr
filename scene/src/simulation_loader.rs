use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::path::Path;

use bubbler::bubbler::Bubbler;
use flate2::Compression;
use flate2::read::ZlibDecoder;
use flate2::write::ZlibEncoder;
use sph::Simulation;

pub fn dump(path: &Path, simulation: &Simulation, bubbler: &Bubbler) -> Result<(), std::io::Error> {
    let buffer = BufWriter::new(File::create(path)?);
    let encoder = ZlibEncoder::new(buffer, Compression::default());
    serde_json::to_writer(encoder, &(simulation, bubbler))?;

    Ok(())
}

pub fn load(path: &Path) -> Result<(Simulation, Bubbler), std::io::Error> {
    let buffer = BufReader::new(File::open(path)?);
    let decoder = ZlibDecoder::new(buffer);
    let (mut simulation, bubbler): (Simulation, Bubbler) = serde_json::from_reader(decoder)?;

    simulation.sync();

    Ok((simulation, bubbler))
}
