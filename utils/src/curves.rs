use serde_derive::*;
use nalgebra::Vector3;

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(tag = "type")]
pub enum Curve {
    #[serde(rename = "linear")]
    Linear {
        a: Vector3<f32>,
        b: Vector3<f32>,
    },
    #[serde(rename = "bezier")]
    Bezier {
        control_points: [Vector3<f32>; 4],
    }
}

impl Curve {
    pub fn sample(&self, t: f32) -> Vector3<f32> {
        match self {
            Curve::Linear { a, b } => (*b - *a) * t + *a,
            Curve::Bezier { control_points } => {
                let tx = 1.0 - t;
                let mut result = Vector3::zeros();

                let len = control_points.len() as i32;
                let mults = [1., 3., 3., 1.];

                for i in 0..len {
                    let m = tx.powi(len - 1 - i) * t.powi(i) * mults[i as usize];
                    result += control_points[i as usize] * m;
                }

                result
            },
        }
    }
}
