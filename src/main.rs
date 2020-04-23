#[macro_use]
extern crate clap;
extern crate render;
extern crate sph_scene;

use clap::App;

use crate::polygonization::main_polygonization;
use crate::simulation::main_simulation;
use crate::viewer::main_viewer;
use crate::raytrace::main_render;

mod polygonization;
mod simulation;
mod viewer;
mod raytrace;
mod pipeline;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let conf = load_yaml!("cli.yml");
    let matches = App::from_yaml(conf).get_matches();

    if let Some(args) = matches.subcommand_matches("simulate") {
        main_simulation(args)?;
    } else if let Some(args) = matches.subcommand_matches("polygonize") {
        main_polygonization(args)?;
    } else if let Some(args) = matches.subcommand_matches("view") {
        main_viewer(args)?;
    } else if let Some(args) = matches.subcommand_matches("render") {
        main_render(args)?;
    } else if let Some(args) = matches.subcommand_matches("pipeline") {
        pipeline::main_pipeline(args)?;
    } else {
        println!("Please refer to the usage:\n{}", matches.usage())
    }

    Ok(())
}

