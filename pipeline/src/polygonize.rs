use std::fs;
use std::path::{Path, PathBuf};

use sph::bubbler::DiffuseParticleType;
use indicatif::{ProgressBar, ProgressStyle};
use nalgebra::Vector3;
use rayon::prelude::*;
use raytracer::{Camera, scene_config};
use raytracer::scene_config::{MeshConfig, BubbleConfig, VolumeConfig, FoamConfig};
use search::HashGrid;
use sph::{Simulation, SimulationFluidSnapshot, Fluid, RigidObject};
use sph_scene::{ConfigurationAnisotropication, Scene, FluidConfiguration};
use utils::kernels::CubicSpine;

use raytracer::Particles;
use mesher::anisotropication::Anisotropicator;
use mesher::interpolation::InterpolationAlgorithms;
use mesher::Mesher;

pub fn snapshot_simulation(simulation: &Simulation, solids: &Vec<RigidObject>, anisotropic_radius: Option<f32>, fluid_idx: usize) -> SimulationFluidSnapshot {
    let all_positions = simulation.positions.read().unwrap();
    let all_densities = simulation.density.read().unwrap();

    let mut particles: Vec<Vector3<f32>> = vec![];
    let mut densities = vec![];

    let mut mass: Option<f32> = None;

    simulation.particles_fluid_type.iter().enumerate()
        .filter(|(_, &idx)| idx == fluid_idx)
        .for_each(|(i, _)| {
            if mass.is_none() { mass = Some(simulation.mass(i)); }

            particles.push(all_positions[i]);
            densities.push(all_densities[i]);
        });

    let mut neighbours_struct = HashGrid::new(simulation.kernel_radius());
    neighbours_struct.insert(&particles);

    let anisotropic_neighbours = if let Some(an_radius) = anisotropic_radius {
        let mut an_grid = HashGrid::new(an_radius);
        an_grid.insert(&particles);
        an_grid.find_all_neighbours(&particles)
    } else { vec![] };

    let solids_advected = simulation.solids().iter().enumerate()
        .map(|(i, s)| {
            let mut s = s.copy();
            s.load_grid_from(&solids[i]); 
            s
        })
        .collect();

    SimulationFluidSnapshot {
        particles,
        densities,
        solids: solids_advected,
        neighbours_struct,
        anisotropic_neighbours,
        kernel: CubicSpine::new(simulation.kernel_radius()),
        mass: mass.unwrap_or(0.1),
    }
}

fn get_interpolation_algorithm(enable_interpolation: bool) -> InterpolationAlgorithms {
    match enable_interpolation {
        false => InterpolationAlgorithms::None,
        true => InterpolationAlgorithms::Linear,
    }
}

fn get_anisotropicator(kernel_radius: f32, enable_anisotropication: bool, conf: &ConfigurationAnisotropication) -> (Option<Anisotropicator>, Option<f32>) {
    let mut anisotropic_radius = None;

    let anisotropicator = match enable_anisotropication {
        true => {
            anisotropic_radius = Some(Anisotropicator::compute_radius(kernel_radius));
            Some(Anisotropicator::new(conf.smoothness, conf.min_nb_neighbours, conf.kr, conf.ks, conf.kn))
        }
        false => None
    };

    (anisotropicator, anisotropic_radius)
}

fn get_simulation_dumps_paths(folder: &Path) -> Result<Vec<PathBuf>, Box<dyn std::error::Error>> {
    let mut files: Vec<PathBuf> = fs::read_dir(folder)?
        .filter_map(Result::ok)
        .filter_map(|d| d.path().to_str().and_then(|f| if f.ends_with(".sim.bin") { Some(d) } else { None }))
        .map(|d| d.path())
        .collect();

    files.sort();

    Ok(files)
}

fn generate_rigid_objects(scene: &Scene, simulation: &Simulation) -> Vec<MeshConfig> {
    let mut result = vec![];
    result.extend(
        (0..simulation.solid_count()).filter_map(|i| {
            let v = simulation.solid(i);
            let conf = &scene.solids[i];

            if conf.display {
                Some(scene_config::MeshConfig {
                    path: scene.solids[i].file(&Path::new(&scene.global_config.data_path)).to_str().unwrap().to_string(),
                    scale: conf.scale(),
                    rotation: v.euler_angle(),
                    position: v.final_position(),
                    override_material: scene.solids[i].material.clone(),
                })
            } else {
                None
            }
        })
    );
    result
}


fn generate_fluid_bubble(conf: &FluidConfiguration, fluid: &Fluid, path: &Path) -> Option<BubbleConfig> {
    if let Some(conf) = &conf.bubbler {
        if conf.bubble.ignore {
            return None;
        }

        assert!(fluid.bubbler().is_some());

        let positions: Vec<Vector3<f32>> = fluid.bubbler().unwrap().read().unwrap().get_particles().iter()
            .filter(|&p| p.kind == DiffuseParticleType::Bubble)
            .map(|p| p.position).collect();

        Particles::new_cst_radius(positions, conf.bubble.radius)
            .dump(path)
            .unwrap();

        Some(BubbleConfig {
            path: path.to_str().unwrap().to_string(),
            material: conf.bubble.material.clone(),
        })

    } else {
        None
    }
}

fn generate_fluid_foam(conf: &FluidConfiguration, fluid: &Fluid, path: &Path)  -> Option<FoamConfig> {
    if let Some(conf) = &conf.bubbler {
        if conf.foam.ignore {
            return None;
        }

        assert!(fluid.bubbler().is_some());

        let positions: Vec<Vector3<f32>> = fluid.bubbler().unwrap().read().unwrap().get_particles().iter()
            .filter(|&p| p.kind == DiffuseParticleType::Foam)
            .map(|p| p.position).collect();

        Particles::new_cst_radius(positions, conf.foam.radius)
            .dump(path)
            .unwrap();

        Some(FoamConfig{
            path: path.to_str().unwrap().to_string(),
            color: conf.foam.color,
            radius: conf.foam.radius,
            density_scaling_factor: conf.foam.density_scaling_factor
        })
    } else {
        None
    }
}


fn generate_fluid_mesh(conf: &FluidConfiguration, simulation: &Simulation, solids: &Vec<RigidObject>, fluid_idx: usize, path: &Path) -> MeshConfig {
    let buffer = &mut fs::File::create(path).unwrap();

    let interpolation_algorithm = get_interpolation_algorithm(conf.meshing.enable_interpolation);
    let (anisotropicator, anisotropic_radius) = get_anisotropicator(simulation.kernel_radius(),
    conf.meshing.enable_anisotropication,
    &conf.meshing.anisotropication_config);

    let snapshot = snapshot_simulation(simulation, solids, anisotropic_radius, fluid_idx);

    let mesher = Mesher::new(conf.meshing.iso_value, conf.meshing.cube_size, interpolation_algorithm, anisotropicator);
    mesher.clone().convert_into_obj(&snapshot, buffer);

    scene_config::MeshConfig {
        path: path.to_str().unwrap().to_string(),
        scale: Vector3::new(1., 1., 1.),
        rotation: Vector3::zeros(),
        position: Vector3::zeros(),
        override_material: conf.material.clone(),
    }
}

fn generate_fluid_objects(scene: &Scene, simulation: &Simulation, solids: &Vec<RigidObject>, dump_directory: &Path, idx: usize) -> Result<Vec<VolumeConfig>, Box<dyn std::error::Error>> {
    let mut result = vec![];

    let fluids_map = scene.load_fluids_map();
    for (name, conf) in &scene.fluids {
        let fluid_idx = *fluids_map.get(name).unwrap();
        let fluid = &simulation.fluids()[fluid_idx];
        let template_path = |s| dump_directory.join(format!("{:08}_{}_{}.obj", idx, name, s));

        let mesh = generate_fluid_mesh(conf, simulation, solids, fluid_idx, &template_path("mesh"));
        let foam = generate_fluid_foam(conf, fluid, &template_path("foam"));
        let bubble = generate_fluid_bubble(conf, fluid, &template_path("bubble"));

        result.push(VolumeConfig {
            mesh,
            foam,
            bubble
        });
    }

    Ok(result)
}

pub fn pipeline_polygonize(scene: &Scene, input_directory: &Path, dump_directory: &Path) -> Result<(), Box<dyn std::error::Error>> {
    fs::create_dir_all(dump_directory)?;

    let simulations: Vec<PathBuf> = get_simulation_dumps_paths(input_directory)?;

    let pb = ProgressBar::new(simulations.len() as u64);
    pb.set_style(ProgressStyle::default_bar()
        .template("[{elapsed_precise}] [{per_sec}] [{eta_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7}"));
    pb.tick();

    let solids = &scene.load_solids()?;

    simulations.par_iter().enumerate().for_each(|(idx, path)| {
        let simulation = match Simulation::load(&path) {
            Err(e) => {panic!(format!("error (path:{:?}): {:?}", path, e))},
            Ok(o) => o
        };

        let path_yaml = dump_directory.join(format!("{:08}.yaml", idx));

        let camera = &simulation.camera;

        let rigid_objects = generate_rigid_objects(scene, &simulation);
        let volumes = generate_fluid_objects(scene, &simulation, &solids, dump_directory, idx).unwrap();

        let config = scene_config::SceneConfig {
            meshes: rigid_objects,
            volumes,
            spheres: Vec::new(),
            lights: Vec::new(),
            camera: Camera::new(camera.position(), camera.up(), camera.forward(),
                                scene.render_config.resolution.0 as f32, scene.render_config.resolution.1 as f32),
        };

        config.save(&path_yaml).unwrap();

        pb.inc(1);
    });

    pb.finish_with_message("polygonization done");

    Ok(())
}
