use std::fs;
use std::path::{Path, PathBuf};
use std::time::Instant;

use clap::ArgMatches;
use kiss3d::camera::camera::Camera;
use nalgebra::{Point3, Vector3};

fn get_meshes_paths(folder: &Path) -> Result<Vec<PathBuf>, Box<dyn std::error::Error>> {
    let mut files: Vec<PathBuf> = fs::read_dir(folder)?
        .filter_map(Result::ok)
        .filter_map(|d| d.path().to_str().and_then(|f| if f.ends_with(".obj") { Some(d) } else { None }))
        .map(|d| d.path())
        .collect();

    files.sort();

    Ok(files)
}

fn view_meshes(meshes_folder: &Path, back_face_culling: bool) -> Result<(), Box<dyn std::error::Error>> {
    let scale = Vector3::new(1., 1., 1.);

    let mut renderer = render::scene::Scene::new(1.);
    renderer.camera.look_at(Point3::new(0.0, 1., -2.), Point3::new(0., 0., 5.)); //FIXME make camera configurable

    let meshes = get_meshes_paths(meshes_folder)?;
    if meshes.len() == 0 {
        println!("No mesh found");
        return Ok(());
    }

    let mut i_obj: usize = 0;

    let mut obj = renderer.window.add_obj(meshes[i_obj].as_ref(), meshes_folder, scale);
    obj.enable_backface_culling(back_face_culling);

    let mut show_info = true;
    let mut pause = true;
    let mut now = Instant::now();

    let mut time_step = 0.;

    let mut dt_i_obj: isize = 0;

    while renderer.render() {
        if !pause && now.elapsed().as_secs_f32() > time_step || dt_i_obj != 0 {
            i_obj = ((if dt_i_obj == 0 { i_obj as isize + 1 } else { i_obj as isize + dt_i_obj } + meshes.len() as isize) % meshes.len() as isize) as usize; // FIXME: beeeuuuuuuuuurk
            dt_i_obj = 0;
            renderer.window.remove_node(&mut obj);
            obj = renderer.window.add_obj(&meshes[i_obj], meshes_folder, scale);
            obj.enable_backface_culling(back_face_culling);

            now = Instant::now();
        }

        for event in renderer.window.events().iter() {
            match event.value {
                render::event::WindowEvent::Key(key, render::event::Action::Release, _) => {
                    match key {
                        render::event::Key::I => {
                            show_info = !show_info;
                        }
                        render::event::Key::Space => {
                            pause = !pause;
                        }
                        render::event::Key::PageUp => {
                            time_step = (time_step + 0.01).min(3.);
                        }
                        render::event::Key::PageDown => {
                            time_step = (time_step - 0.01).max(0.);
                        }
                        _ => {}
                    }
                }
                render::event::WindowEvent::Key(key, render::event::Action::Press, _) => {
                    match key {
                        render::event::Key::F => {
                            dt_i_obj = 1;
                        }
                        render::event::Key::S => {
                            dt_i_obj = -1;
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        if show_info {
            renderer.debug_text(&format!("\
                iteration: {}/{}\n\
                dt: {:.3}\n\
                eye: {}\
                ", i_obj, meshes.len() - 1, time_step, renderer.camera.eye()));
        }
    }

    Ok(())
}

pub fn main_viewer(args: &ArgMatches) -> Result<(), Box<dyn std::error::Error>> {
    let meshes_folder = Path::new(args.value_of("meshes_folder").unwrap());
    let back_face_culling = args.is_present("back_face_culling");

    view_meshes(meshes_folder, back_face_culling)?;

    Ok(())
}

