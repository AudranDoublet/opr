extern crate pipeline;

use std::path::Path;

use clap::ArgMatches;

pub fn main_pipeline(args: &ArgMatches) -> Result<(), Box<dyn std::error::Error>> {
    let scene_file = args.value_of("scene").unwrap();
    let output_directory= &Path::new(args.value_of("output_directory").unwrap());

    let scene_c = sph_scene::load_scene(scene_file)?;
    pipeline::PipelineStep::parse_step(args.value_of("from")).run_all(&scene_c, output_directory)
}
