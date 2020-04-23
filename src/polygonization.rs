extern crate sph_scene;
extern crate pipeline;

use clap::ArgMatches;
use std::path::Path;

pub fn main_polygonization(args: &ArgMatches) -> Result<(), Box<dyn std::error::Error>> {
    let output_directory = match args.value_of("output_directory") {
        Some(d) => Path::new(d),
        None => Path::new("./")
    };

    let scene_file = args.value_of("scene").unwrap();
    let dump_directory = Path::new(args.value_of("dump_directory").unwrap());
    let disable_interpolation = args.is_present("disable_interpolation");
    let disable_anisotropication = args.is_present("disable_anisotropication");

    let mut scene_c = sph_scene::load_scene(scene_file)?;

    scene_c.meshing_config.enable_anisotropication  = !disable_anisotropication;
    scene_c.meshing_config.enable_interpolation     = !disable_interpolation;

    pipeline::polygonize::pipeline_polygonize(&scene_c, dump_directory, output_directory)
}
