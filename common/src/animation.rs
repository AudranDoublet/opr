use nalgebra::Vector3;
use serde_derive::*;

#[derive(Deserialize, Debug, Clone)]
pub enum VariableType {
    AngularVelocity,
    Velocity,
    AngularAcceleration,
    Acceleration,
    Position,
    Rotation,
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
    #[serde(rename = "lerp")]
    Evolution {
        variable: VariableType,
        to_value: Vector3<f32>,
        time: f32,

        // states variable
        #[serde(skip_deserializing)]
        current_time: f32,
    },
    #[serde(rename = "lookat")]
    LookAt {
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

    if *current > max {
        *dt = *current - max;
        *current = max;
    } else {
        *dt = 0.0;
    }

    *current = next;
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

    pub fn step(&mut self, handler: &mut dyn AnimationHandler, mut dt: f32) -> f32 {
        match self {
            Animation::Steps { loop_num, steps, current, .. } => {
                while *current < steps.len() && dt > 0.0 {
                    dt = steps[*current].step(handler, dt);

                    if dt > 0.0 {
                        *current += 1;
                    }
                }

                if *current > steps.len() {
                    *loop_num -= 1;

                    if *loop_num == 0 {
                        dt
                    } else {
                        steps.iter_mut().for_each(|v| v.reset());
                        self.step(handler, dt)
                    }
                } else {
                    dt
                }
            },
            Animation::Group { elements } => {
                elements.iter_mut().map(|v| v.step(handler, dt)).fold(0.0, |a, b| a.max(b))
            },
            Animation::Constant { variable, value, time, current_time } => {
                timer(current_time, *time, &mut dt);
                handler.set_variable(variable, *value);

                dt
            },
            Animation::Set { variable, value } => {
                handler.set_variable(variable, *value);
                dt
            },
            Animation::Evolution { variable, to_value, time, current_time } => {
                if current_time >= time {
                    dt
                } else {
                    let value = handler.get_variable(variable);
                    let old = timer(current_time, *time, &mut dt);

                    handler.set_variable(variable,
                                         value + (*to_value - value)
                                            * (*current_time - old) / (*time - old));

                    dt
                }
            },
            Animation::LookAt { position } => {
                handler.look_at(*position);
                dt
            },
            Animation::Blank => 0.0,
        }
    }

    pub fn animate(&mut self, handler: &mut dyn AnimationHandler, dt: f32, reset_at_end: bool) {
        loop {
            let dt = self.step(handler, dt);

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
