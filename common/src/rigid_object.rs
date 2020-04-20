extern crate rayon;

use std::sync::Mutex;

use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

use crate::{DiscreteGrid, mesh::MassProperties, search::BVH, search::Sphere};
use crate::mesher::types::{VertexWorld};

#[derive(Serialize, Deserialize, Debug)]
pub struct RigidObject
{
    pub dynamic: bool,

    /** State variables */
    position: Vector3<f32>,
    pub velocity: Vector3<f32>,
    angular_velocity: Vector3<f32>,
    rotation: Quaternion<f32>,

    /** Geometric mesh-constants */
    body_inertia_tensor: Matrix3<f32>,
    inv_body_inertia_tensor: Matrix3<f32>,
    mass: f32,
    pub center_of_mass: Vector3<f32>,

    #[serde(skip_serializing, skip_deserializing)]
    grid: DiscreteGrid,
    #[serde(skip_serializing, skip_deserializing)]
    bvh: BVH<Sphere>,
    #[serde(skip_serializing, skip_deserializing)]
    volume: Vec<f32>,
    #[serde(skip_serializing, skip_deserializing)]
    boundary_x: Vec<Vector3<f32>>,
    #[serde(skip_serializing, skip_deserializing)]
    forces: Vec<Mutex<Vector3<f32>>>,
    #[serde(skip_serializing, skip_deserializing)]
    torques: Vec<Mutex<Vector3<f32>>>,
}

#[derive(Debug)]
pub struct Constraint
{
    pub cp0: Vector3<f32>,
    pub cp1: Vector3<f32>,
    normal: Vector3<f32>,
    tangent: Vector3<f32>,
    nkn_inv: f32,
    depth: f32,
    p_max: f32,
    goal_u_rel_n: f32,
    sum: f32,
}

/**
 * Reference for rigid object physic: http://www.cs.cmu.edu/~baraff/sigcourse/notesd1.pdf
 */
impl RigidObject
{
    pub fn new(grid: DiscreteGrid, dynamic: bool, properties: MassProperties) -> RigidObject {
        let mut res = RigidObject {
            dynamic: dynamic,

            // state
            position: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            velocity: Vector3::zeros(),
            rotation: Quaternion::new(1.0, 0.0, 0.0, 0.0),

            grid,
            bvh: BVH::build(&vec![]),
            volume: Vec::new(),
            boundary_x: Vec::new(),
            forces: Vec::new(),
            torques: Vec::new(),

            center_of_mass: properties.center_of_mass,
            body_inertia_tensor: properties.inertia,
            inv_body_inertia_tensor: properties.inertia.try_inverse().expect("can't inverse inertia tensor"),
            mass: properties.mass,
        };

        res.bvh = res.to_particle_tree(0.01);
        res
    }

    pub fn to_particles(&self, radius: f32) -> Vec<VertexWorld> {
        let (min, max) = self.grid.get_domain_definition();
        let step = (max - min) / radius;

        let mut res = vec![];
        for z in 0..step.z as u32 {
            for y in 0..step.y as u32 {
                for x in 0..step.x as u32 {
                    let pos = min + VertexWorld::new(x as f32, y as f32, z as f32) * radius;
                    let v = self.grid.interpolate(0, pos, false);

                    if let Some((d, _)) = v {
                        if d <= 0.00 {
                            res.push(pos);
                        }
                    }
                }
            }
        }

        res
    }

    pub fn to_particle_tree(&self, radius: f32) -> BVH<Sphere> {
        BVH::build(
            &self.to_particles(radius).iter()
                .map(|p| Sphere::new(p, radius))
                .collect()
        )
    }

    pub fn reset_force_and_torque(&mut self)
    {
        self.forces.clear();
        self.forces.resize_with(rayon::current_num_threads(), || Mutex::new(Vector3::zeros()));

        self.torques.clear();
        self.torques.resize_with(rayon::current_num_threads(), || Mutex::new(Vector3::zeros()));
    }

    pub fn get_force_and_torque(&self) -> (Vector3<f32>, Vector3<f32>)
    {
        let force = self.forces.iter().map(|v| *v.lock().unwrap()).sum::<Vector3<f32>>();
        let torque = self.torques.iter().map(|v| *v.lock().unwrap()).sum::<Vector3<f32>>();

        (force, torque)
    }

    pub fn add_force(&self, position: Vector3<f32>, force: Vector3<f32>)
    {
        if !self.dynamic {
            return;
        }

        let tid = rayon::current_thread_index().unwrap_or(0);

        let mut v_force = self.forces[tid].lock().unwrap();
        let mut v_torque = self.torques[tid].lock().unwrap();

        *v_force += force;
        *v_torque += (position - self.center_position()).cross(&force);
    }

    /**
     * Compute rotation matrix from rotation quaternion
     */
    pub fn rotation_matrix(&self) -> Matrix3<f32> {
        let s = self.rotation.w;
        let vx = self.rotation.i;
        let vy = self.rotation.j;
        let vz = self.rotation.k;

        Matrix3::new(
            1. - 2. * vy * vy - 2. * vz * vz, 2. * vx * vy - 2. * s * vz, 2. * vx * vz + 2. * s * vy,
            2. * vx * vy + 2. * s * vz, 1. - 2. * vx * vx - 2. * vz * vz, 2. * vy * vz - 2. * s * vx,
            2. * vx * vz - 2. * s * vy, 2. * vy * vz + 2. * s * vx, 1. - 2. * vx * vx - 2. * vy * vy,
        )
    }

    /**
     * Compute inverse of inertia tensor relativly to current rotation
     */
    pub fn inv_inertia_tensor(&self) -> Matrix3<f32> {
        let r = self.rotation_matrix();
        r * self.inv_body_inertia_tensor * r.transpose()
    }

    pub fn compute_k_matrix(&self, contact_point: Vector3<f32>) -> Matrix3<f32> {
        if !self.dynamic {
            Matrix3::zeros()
        } else {
            let contact_point = self.position_in_mesh_space(contact_point);
            let r = contact_point.cross_matrix();

            Matrix3::identity() / self.mass + r.transpose() * self.inv_inertia_tensor() * r
        }
    }

    pub fn update(&mut self, gravity: Vector3<f32>, dt: f32) {
        if !self.dynamic {
            return;
        }

        let (mut force, torque) = self.get_force_and_torque();

        force += gravity * self.mass;

        self.velocity += (force / self.mass) * dt;
        self.position += self.velocity * dt;

        self.angular_velocity += self.inv_inertia_tensor() * torque * dt;

        let q = Quaternion::new(0.0, self.angular_velocity.x, self.angular_velocity.y, self.angular_velocity.z);

        self.rotation += (dt / 2.) * (q * self.rotation);
        self.rotation.normalize_mut();

        self.reset_force_and_torque();
    }

    pub fn update_vel(&mut self, dt: f32) {
        if !self.dynamic {
            return;
        }

        let (force, torque) = self.get_force_and_torque();

        let vdiff = force / self.mass * dt;
        let avdiff = self.inv_inertia_tensor() * torque * dt;

        self.velocity += vdiff;
        self.position += vdiff * dt;
        self.angular_velocity += avdiff;

        let q = Quaternion::new(0.0, avdiff.x, avdiff.y, avdiff.z);

        self.rotation += (dt / 2.) * (q * self.rotation);
        self.rotation.normalize_mut();

        self.reset_force_and_torque();
    }

    pub fn point_velocity(&self, x: Vector3<f32>) -> Vector3<f32> {
        self.angular_velocity.cross(&(x - self.position())) + self.velocity
    }

    pub fn pred_point_velocity(&self, x: Vector3<f32>) -> Vector3<f32> {
        let (force, torque) = self.get_force_and_torque();

        let vdiff = force / self.mass;
        let avdiff = self.inv_inertia_tensor() * torque;

        (self.angular_velocity + avdiff).cross(&(x - self.position())) + self.velocity + vdiff
    }

    pub fn rotation(&self) -> UnitQuaternion<f32> {
        UnitQuaternion::from_quaternion(self.rotation)
    }

    pub fn center_position(&self) -> Vector3<f32> {
        self.position
    }

    pub fn position(&self) -> Vector3<f32> {
        self.position
    }

    pub fn set_position(&mut self, position: Vector3<f32>) {
        self.position = position + self.center_of_mass
    }

    pub fn particle_volume(&self, i: usize) -> f32 {
        self.volume[i]
    }

    pub fn particle_boundary_x(&self, i: usize) -> Vector3<f32> {
        self.boundary_x[i]
    }

    pub fn position_in_mesh_space(&self, p: Vector3<f32>) -> Vector3<f32>
    {
        let r = self.rotation_matrix();
        r.transpose() * (p - self.position())
    }

    pub fn position_in_world_space(&self, p: Vector3<f32>) -> Vector3<f32> {
        self.rotation_matrix() * p + self.position()
    }

    pub fn set_volume_and_boundary_x(&mut self, volume: Vec<f32>, boundary_x: Vec<Vector3<f32>>)
    {
        self.volume = volume;
        self.boundary_x = boundary_x;
    }

    pub fn is_inside(&self, position: Vector3<f32>, eps: f32) -> Option<(f32, Vector3<f32>)> {
        if let Some((dist, normal)) = self.grid.interpolate(0, self.position_in_mesh_space(position), true) {
            if dist < eps {
                Some((dist, normal))
            } else {
                None
            }
        } else {
            None
        }
    }

    /*
    fn prepare_constraint(&self, other: &RigidObject,
                          depth: f32,
                          cp0: Vector3<f32>, cp1: Vector3<f32>,
                          normal: Vector3<f32>) -> Constraint {
        let restitution = 0.1; //FIXME

        let v0 = self.pred_point_velocity(cp0);
        let v1 = other.pred_point_velocity(cp1);

        let u_rel = v0 - v1;
        let u_rel_n = normal.dot(&u_rel);

        let k = self.compute_k_matrix(cp0) + other.compute_k_matrix(cp1);
        let mut tangent = u_rel - u_rel_n * normal;

        if tangent.norm_squared() > 1e-6 {
            tangent.normalize_mut();
        }

        Constraint {
            cp0: cp0,
            cp1: cp1,
            normal: normal,
            tangent: tangent,
            depth: depth,
            nkn_inv: 1.0 / normal.dot(&(k * normal)),
            p_max: 1.0 / tangent.dot(&(k * tangent)) * u_rel.dot(&tangent),
            goal_u_rel_n: match u_rel_n {
                v if v < 0.0 => -restitution * v,
                _ => 0.0,
            },
            sum: 0.0,
        }
    }

    fn solve_constraint(&self, other: &RigidObject, constraint: &mut Constraint) {
        // let p = self.dynamic && other.dynamic;

        let stiffness = 1.1;
        let friction = 0.5;

        let v0 = self.pred_point_velocity(constraint.cp0);
        let v1 = other.pred_point_velocity(constraint.cp1);

        let u_rel = v0 - v1;
        let u_rel_n = u_rel.dot(&constraint.normal);

        let delta_u_rel_n = constraint.goal_u_rel_n - u_rel_n;

        let mut correction_magnitude = (constraint.nkn_inv * delta_u_rel_n).max(-constraint.sum);

        if constraint.depth < 0.0 {
            correction_magnitude -= stiffness * constraint.nkn_inv * constraint.depth;
        }

        constraint.sum += correction_magnitude;

        let mut force = correction_magnitude * constraint.normal;

        // friction
        force += match correction_magnitude.abs() * friction {
            v if v > constraint.p_max => -constraint.p_max * constraint.tangent,
            v if v < -constraint.p_max => constraint.p_max * constraint.tangent,
            v => -v * constraint.tangent,
        };

        self.add_force(constraint.cp0, force);
        other.add_force(constraint.cp1, -force);
    }
    */

    /**
     * Reference:
     * A review of computer simulation of tumbling mills by the discrete element method: Part I—contact mechanics
     */
    pub fn collide(&self, other: &RigidObject) -> Vec<Vector3<f32>> {
        if !self.dynamic && !other.dynamic {
            return vec![];
        }

        let d: f32 = 0.02;

        let k = 1.7;
        let damping_coeff = 0.5;
        let t = 1.2;

        let r_inv = self.rotation_matrix().transpose();

        let rotation = r_inv * other.rotation_matrix();
        let translation = r_inv * (other.position() - self.position());

        let collisions = self.bvh.intersects(&other.bvh, &rotation, &translation);

        let v = collisions.iter()
            .map(|(a, b)| {
                let pa = self.position_in_world_space(a.position);
                let pb = other.position_in_world_space(b.position);

                let r_ij = pb - pa;
                let v_ij = - self.point_velocity(pa) + other.point_velocity(pb);

                let r_ij_n = r_ij.norm();

                if r_ij_n > d {
                    return Vector3::zeros();
                }

                let r_ij = r_ij / r_ij_n;
                let v_ij_t = v_ij.dot(&r_ij) * r_ij;
                let v_ij = v_ij - v_ij_t; //(v_ij.dot(&r_ij) * r_ij);

                let f_s = -k * (d - r_ij_n) * r_ij;
                let f_d = damping_coeff * v_ij;
                let f_t = t * v_ij_t;
                let f = f_s + f_d + f_t;

                self.add_force(pa, f);
                other.add_force(pb, -f);

                pa
            }).collect();

        v
    }

    pub fn compute_volume_and_boundary_x(&self, position: &mut Vector3<f32>, velocity: &mut Vector3<f32>, particle_radius: f32, kernel_radius: f32, dt: f32) -> (f32, Vector3<f32>)
    {
        let x = self.position_in_mesh_space(*position);

        let r = self.rotation_matrix();

        let mut volume = 0.0;
        let mut boundary_x = Vector3::zeros();

        let (dist, normal) = self.grid.interpolate_or(0, x, true);

        if dist > 0.1 * particle_radius && dist < kernel_radius {
            if let Some((v, _)) = self.grid.interpolate(1, x, false) {
                if v > 1e-5 { // != 0
                    volume = v;

                    // unrotate normal
                    let normal = r * normal;

                    if normal.norm() > 1e-5 { // != 0
                        boundary_x = *position - dist * normal.normalize();
                    }
                }
            }
        } else if dist < 0.1 * particle_radius {
            // unrotate normal
            let normal = r * normal;

            if normal.norm() > 1e-5 { // != 0
                // particle is 'in' surface, update position and velocity in order to fix
                // them
                let normal = normal.normalize();

                let d = (-dist).min(50. * particle_radius * dt);
                *position += d * normal;
                *velocity += (0.05 - velocity.dot(&normal)) * normal;
            }
        }

        (volume, boundary_x)
    }
}
