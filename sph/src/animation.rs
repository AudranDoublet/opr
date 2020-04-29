use nalgebra::Vector3;
use serde_derive::*;

use utils::Curve;

#[derive(Deserialize, Debug, Clone)]
pub enum VariableType {
    AngularVelocity,
    Velocity,
    AngularAcceleration,
    Acceleration,
    Position,
    Rotation,
}

fn smooth_strength() -> f32 {
    2.
}

#[derive(Deserialize, Debug, Clone)]
pub struct Smoothing {
    begin: f32,
    end: f32,
    #[serde(default = "smooth_strength")]
    begin_strength: f32,
    #[serde(default = "smooth_strength")]
    end_strength: f32,
}

impl Smoothing {
    fn smooth_begin(&self, t: f32) -> f32 {
        t*t
    }

    fn smooth_end(&self, t: f32) -> f32 {
        1. - (1. - t).powi(2)
    }

    pub fn smooth(&self, t: f32) -> f32 {
        let begin_perc = self.begin / self.begin_strength;
        let end_perc = (1. - self.end) / self.end_strength;

        if t < self.begin {
            let scaled = self.smooth_begin(t / self.begin);

            begin_perc * scaled
        } else if t > self.end {
            let scaled = self.smooth_end((t - self.end) / (1. - self.end));

            end_perc * scaled + (1. - end_perc)
        } else {
            (t  - self.begin) / (self.end - self.begin) * (1.0 - begin_perc - end_perc) + begin_perc
        }
    }
}

#[derive(Deserialize, Debug, Clone)]
#[serde(tag = "type")]
pub enum Animation {
    #[serde(rename = "steps")]
    Steps {
        loop_count: isize,
        steps: Vec<Animation>,

        // states variable
        #[serde(skip_deserializing)]
        current: usize,
        #[serde(skip_deserializing)]
        loop_num: isize,
    },
    #[serde(rename = "group")]
    Group {
        elements: Vec<Animation>,
    },
    #[serde(rename = "constant")]
    Constant {
        variable: VariableType,
        value: Vector3<f32>,

        time: f32,

        // states variable
        #[serde(skip_deserializing)]
        current_time: f32,
    },
    #[serde(rename = "set")]
    Set {
        variable: VariableType,
        value: Vector3<f32>,
    },
    #[serde(rename = "curve")]
    Evolution {
        variable: VariableType,
        curve: Curve,
        time: f32,

        #[serde(default)]
        smoothing: Option<Smoothing>,

        // states variable
        #[serde(skip_deserializing)]
        current_time: f32,
    },
    #[serde(rename = "lookat")]
    LookAt {
        position: Vector3<f32>,
    },
    #[serde(rename = "lookat_relative")]
    LookAtRelative {
        position: Vector3<f32>,
    },
    Blank,
}

pub trait AnimationHandler {
    fn get_variable(&self, variable: &VariableType) -> Vector3<f32>;

    fn set_variable(&mut self, variable: &VariableType, value: Vector3<f32>);

    fn look_at(&mut self, at: Vector3<f32>);
}

fn timer(current: &mut f32, max: f32, dt: &mut f32) -> f32 {
    let next = *current + *dt;
    let old = *current;

    if next > max {
        *dt = *current - max;
        *current = max;
    } else {
        *dt = 0.0;
        *current = next;
    }

    old
}

impl Animation {
    pub fn reset(&mut self) {
        match self {
            Animation::Steps { loop_num, loop_count, steps, current, .. } => {
                *loop_num = *loop_count;
                *current = 0;

                steps.iter_mut().for_each(|v| v.reset())
            },
            Animation::Group { elements } => {
                elements.iter_mut().for_each(|v| v.reset())
            },
            Animation::Constant { current_time, .. } => {
                *current_time = 0.0
            },
            Animation::Evolution { current_time, .. } => {
                *current_time = 0.0
            },
            _ => (),
        }
    }

    pub fn step(&mut self, handler: &mut dyn AnimationHandler, mut dt: f32) -> Option<f32> {
        match self {
            Animation::Steps { loop_num, steps, current, .. } => {
                while *current < steps.len() && dt > 0.0 {
                    dt = steps[*current].step(handler, dt).unwrap_or(dt);

                    if dt > 0.0 {
                        *current += 1;
                    }
                }

                if *current > steps.len() {
                    *loop_num -= 1;

                    if *loop_num == 0 {
                        Some(dt)
                    } else {
                        steps.iter_mut().for_each(|v| v.reset());
                        self.step(handler, dt)
                    }
                } else {
                    Some(dt)
                }
            },
            Animation::Group { elements } => {
                Some(
                    elements.iter_mut()
                        .map(|v| v.step(handler, dt).unwrap_or(0.0))
                        .fold(0.0, |a, b| a.max(b))
                )
            },
            Animation::Constant { variable, value, time, current_time } => {
                timer(current_time, *time, &mut dt);
                handler.set_variable(variable, *value);

                Some(dt)
            },
            Animation::Set { variable, value } => {
                handler.set_variable(variable, *value);
                None
            },
            Animation::Evolution { variable, curve, time, current_time, smoothing } => {
                if current_time >= time {
                    Some(dt)
                } else {
                    timer(current_time, *time, &mut dt);

                    let mut time = *current_time / *time;

                    if let Some(s) = smoothing {
                        time = s.smooth(time);
                    }

                    /*
                    let scale = 5e-6;

                    let time = scale * 2. * *current_time / *time - scale;
                    let time = 1. / (1. + (-1000000. * time).exp());
                    */

                    let value = curve.sample(time);
                    handler.set_variable(variable, value);

                    Some(dt)
                }
            },
            Animation::LookAt { position } => {
                handler.look_at(*position);
                None
            },
            Animation::LookAtRelative { position } => {
                let pos = handler.get_variable(&VariableType::Position);

                handler.look_at(pos + *position);
                None
            },
            Animation::Blank => Some(0.0),
        }
    }

    pub fn animate(&mut self, handler: &mut dyn AnimationHandler, dt: f32, reset_at_end: bool) {
        loop {
            let dt = self.step(handler, dt).unwrap_or(0.0);

            if dt > 0.0 {
                if reset_at_end {
                    self.reset();
                } else {
                    *self = Animation::Blank;
                    break;
                }
            } else {
                break;
            }
        }
    }
}

impl Default for Animation {
    fn default() -> Animation {
        Animation::Blank
    }
}
