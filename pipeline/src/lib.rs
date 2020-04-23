pub mod simulate;
pub mod polygonize;
pub mod render;
pub mod video;

use std::path::Path;
use sph_scene::Scene;

pub enum PipelineStep {
    Simulate,
    Polygonize,
    Render,
    Video,
}

impl PipelineStep {
    pub fn parse_step(s: Option<&str>) -> PipelineStep {
        if let Some(s) = s {
            match s.to_lowercase().as_str() {
                "simulate"      => PipelineStep::Simulate,
                "polygonize"    => PipelineStep::Polygonize,
                "mesh"          => PipelineStep::Polygonize,
                "render"        => PipelineStep::Render,
                "video"         => PipelineStep::Video,
                _               => panic!(format!("unknown step `{}`", s)),
            }
        } else {
            PipelineStep::Simulate
        }
    }

    pub fn next(&self) -> Option<PipelineStep> {
        match self {
            PipelineStep::Simulate      => Some(PipelineStep::Polygonize),
            PipelineStep::Polygonize    => Some(PipelineStep::Render),
            PipelineStep::Render        => Some(PipelineStep::Video),
            _ => None,
        }
    }

    pub fn run(&self, scene: &Scene, path: &Path) -> Result<(), Box<dyn std::error::Error>> {
        Ok(match self {
            PipelineStep::Simulate      => simulate::pipeline_simulate(scene, &path.join("simulation"))?,
            PipelineStep::Polygonize    => polygonize::pipeline_polygonize(scene, &path.join("simulation"), &path.join("mesh"))?,
            PipelineStep::Render        => render::pipeline_render(scene, &path.join("mesh"), &path.join("render"))?,
            PipelineStep::Video         => video::pipeline_video(scene, &path.join("render"), &path.join("output.mp4"))?,
        })
    }

    pub fn run_all(&self, scene: &Scene, path: &Path) -> Result<(), Box<dyn std::error::Error>> {
        self.run(scene, path)?;

        if let Some(next) = self.next() {
            next.run_all(scene, path)?;
        }

        Ok(())
    }
}
