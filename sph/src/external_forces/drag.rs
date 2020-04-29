use nalgebra::Vector3;
use rayon::prelude::*;

use crate::Fluid;
use crate::{Simulation, external_forces::ExternalForce};

const PI: f32 = std::f32::consts::PI;
const RHO_A: f32 = 1.2041;
const MU_A: f32 = 0.00001845;
const MU_L: f32 = 0.00102;
const C_B: f32 = 0.5;
const C_D: f32 = 5.0;
const C_F: f32 = 1./3.;
const C_K: f32 = 8.0;
const SIGMA: f32 = 0.0724;
const MAX_NEIGHBOUR: f32 = 38. * 2./3.; // 2/3 of max neighbour count

/*
 * Reference: Approximate Air-Fluid Interactions for SPH (C. Glisser, 2017)
 */
pub struct DragForce {
    air_velocity: Vector3<f32>,
    drag_coefficient: f32,
}

impl DragForce {
    pub fn new(air_velocity: Vector3<f32>, drag_coefficient: f32) -> Box<DragForce> {
        Box::new(DragForce {
            air_velocity: air_velocity,
            drag_coefficient: drag_coefficient,
        })
    }
}

impl ExternalForce for DragForce {
    fn init(&mut self, _: &Fluid, _: &Simulation) { }

    fn compute_acceleration(&self, fluid: &Fluid, sim: &Simulation, accelerations: &mut Vec<Vector3<f32>>) -> Option<f32> {
        let positions = sim.positions.read().unwrap();
        let velocities = sim.velocities.read().unwrap();

        let diam = sim.particle_radius() * 2.;
        let l = (0.75 / PI).cbrt() * diam;

        let inv_td = C_D / 2. * MU_L / (fluid.rest_density() * l*l);
        let td = 1. / inv_td;

        let omega_sq = (C_K * SIGMA / (fluid.rest_density() * l.powi(3)) - inv_td.powi(2)).max(0.0);
        let omega = omega_sq.sqrt();

        // equation 6
        let t_max = (((td.powi(2) * omega_sq + 1.).sqrt() + td * omega).clamp(-0.5 * PI, 0.5 * PI).atan() - PI) / omega;

        // equation 7
        let c_def = 1.0 - (-t_max / td).exp() * ((omega*t_max).cos() + (omega*t_max).sin() / (omega*td));
        let wei = RHO_A * l / SIGMA;

        // equation 8
        let y_coeff = (C_F * wei * c_def) / (C_K * C_B);

        fluid.filter_m(sim, accelerations.par_iter_mut())
             .for_each(|(i, v)| {
                let vi_rel = self.air_velocity - velocities[i];
                let vi_rel_norm_sq = vi_rel.norm_squared();
                let vi_rel_norm = vi_rel_norm_sq.sqrt();

                if vi_rel_norm <= 1e-6 {
                    return;
                }

                let vi_rel_n = vi_rel / vi_rel_norm;

                // compute deformation, equation 8
                let yi_max = (vi_rel_norm_sq * y_coeff).min(1.0);
                let re_i = 2. * ((RHO_A * vi_rel_norm * l) / MU_A).max(0.1);

                let c_di_sphere = match re_i {
                    re_i if re_i <= 1000. => 24. / re_i * (1. + 1./6. * re_i.powf(2./3.)),
                    _                     => 0.424,
                };

                // equation 9
                let c_di_liu = c_di_sphere * (1. + 2.632 * yi_max);

                // compute drag coefficient, equation 10
                let neighbours_count = sim.neighbours_count_with_solids(i);
                let neighbour_perc = (neighbours_count as f32).min(MAX_NEIGHBOUR) / MAX_NEIGHBOUR;

                let c_di = (1.0 - neighbour_perc) * c_di_liu + neighbour_perc;

                // equation 12
                let ai_dropplet = PI * (l + C_B*l*yi_max).powi(2);

                // compute unoccluded area, equation 13
                let ai_unoccluded = (1. - neighbour_perc) * ai_dropplet + neighbour_perc * diam.powi(2);

                // compute occlusion, equation 15
                let w_i = (1.0 - sim.neighbours_reduce_f(true, i, &|r, i, j| {
                    let xij = positions[i] - positions[j];

                    r.max(vi_rel_n.dot(&xij))
                })).clamp(0.0, 1.0);

                // equation 14
                let a_i = w_i * ai_unoccluded;

                // compute force
                let force = self.drag_coefficient * 0.5 * RHO_A * (vi_rel * vi_rel_norm) * c_di * a_i;
                *v += force / sim.mass(i);
            });

        None
    }
}
