use sph_scene::Scene;

use std::path::Path;
use std::process::Command;

pub fn pipeline_video(scene: &Scene, input_directory: &Path, dump_directory: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let fps = scene.simulation_config.fps as usize;

    if dump_directory.exists() {
        std::fs::remove_file(dump_directory)?;
    }

    Command::new("ffmpeg")
            .args(&[
                  "-framerate", fps.to_string().as_str(),
                  "-i", format!("{}/%08d.png", input_directory.to_str().unwrap()).as_str(),
                  "-c:v", "libx264", "-profile:v", "high",
                  "-crf", "20", "-pix_fmt", "yuv420p",
                  dump_directory.to_str().unwrap(),
            ])
            .output()?;

    Ok(())
}
