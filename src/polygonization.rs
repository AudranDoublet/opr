extern crate pipeline;
extern crate sph_scene;

use std::path::Path;

use clap::ArgMatches;

pub fn main_polygonization(args: &ArgMatches) -> Result<(), Box<dyn std::error::Error>> {
    let output_directory = match args.value_of("output_directory") {
        Some(d) => Path::new(d),
        None => Path::new("./")
    };

    let scene_file = args.value_of("scene").unwrap();
    let dump_directory = Path::new(args.value_of("dump_directory").unwrap());
    let disable_interpolation = args.is_present("disable_interpolation");
    let disable_anisotropication = args.is_present("disable_anisotropication");
    let cst_meshing_iso_value = args.value_of("iso_value").unwrap_or("0.05").parse::<f32>().unwrap();
    let cst_meshing_cube_size = args.value_of("cube_size").unwrap_or("0.04").parse::<f32>().unwrap();

    let mut scene_c = sph_scene::load_scene(scene_file)?;

    scene_c.meshing_config.fluid.enable_anisotropication = !disable_anisotropication;
    scene_c.meshing_config.fluid.enable_interpolation = !disable_interpolation;
    scene_c.meshing_config.fluid.iso_value = cst_meshing_iso_value;
    scene_c.meshing_config.fluid.cube_size = cst_meshing_cube_size;

    pipeline::polygonize::pipeline_polygonize(&scene_c, dump_directory, output_directory)
}
