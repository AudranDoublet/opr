use nalgebra::{Matrix3, SVD, U3, Vector3};

use crate::mesher::types::{FluidSnapshot, VertexWorld};

const CST_EPSILON: f32 = 1.0e-5;

#[derive(Clone)]
pub struct Anisotropicator {
    // x_i' = (1-λ)*x_i + λ*sum(W_ij*x_j)/sum(W_ij)
    smoothed_positions: Vec<VertexWorld>,
    // x^{w}_{i} = sum(W_ij*x_j)/sum(W_ij)
    weighted_means: Vec<VertexWorld>,

    // inverse of the radius of the simulation smoothing kernel
    kernel_radius_inv: f32,

    // radius of the isotropic weighting function (usually 2*h with `h` the smoothing radius of the kernel of the particle)
    radius: f32,
    radius_sq: f32,

    cst_smoothness: f32, // λ ~= 0.9

    cst_min_nb_neighbours: usize,

    // k_r > 1. used to prevent extreme deformation (σ_k = max(σ_k, σ_1/k_r))
    cst_kr: f32,

    // k_s such that ||k_s * C|| ~= 1 => to avoid ||C|| explosion when the neighborhood is full.
    // Considering the fact that the covariance matrix C contains small values of the order of the cube of the smoothing radius h^3),
    // so a correct value k_s should be of the order of h^(-3) (ex: 1400 in the paper) so ||k_s * C|| ~= 1
    cst_ks: f32,

    // k_n such that, on too low neighborhood, Σ~ = k_n * I. Should be similar to the kernel radius
    cst_kn_inv: Matrix3<f32>,
}

impl Anisotropicator {
    pub fn new(cst_smoothness: f32, cst_min_nb_neighbours: usize, cst_kr: f32, cst_ks: f32, cst_kn: f32) -> Anisotropicator {
        assert!(0. <= cst_smoothness && cst_smoothness <= 1.);

        Anisotropicator {
            smoothed_positions: vec![],
            weighted_means: vec![],

            kernel_radius_inv: -1.,
            radius: -1.,
            radius_sq: -1.,

            cst_smoothness,
            cst_min_nb_neighbours,
            cst_kr,
            cst_ks,

            cst_kn_inv: (Matrix3::identity() * cst_kn).pseudo_inverse(CST_EPSILON).unwrap(),
        }
    }

    /// isotropic weighting function 2 particles at positions x_i and x_j
    /// * `n_sq`: ||x_j - x_i||^2
    fn w(&self, n_sq: f32) -> f32 {
        if n_sq < self.radius_sq {
            1. - (n_sq.sqrt() / self.radius).powi(3)
        } else {
            0.
        }
    }

    pub fn compute_radius(kernel_radius: f32) -> f32 {
        2. * kernel_radius
    }

    fn init_kernels_centers(&mut self, snapshot: &Box<dyn FluidSnapshot>) {
        let kernel = snapshot.get_kernel();

        assert!(self.cst_ks.log10() >= kernel.radius().log10());

        self.kernel_radius_inv = 1. / kernel.radius();
        self.radius = Anisotropicator::compute_radius(kernel.radius());
        self.radius_sq = self.radius * self.radius;

        self.smoothed_positions.resize(snapshot.len(), Vector3::zeros());
        self.weighted_means.resize(snapshot.len(), Vector3::zeros());

        for i in 0..snapshot.len() {
            let x_i = &snapshot.position(i);

            let mut sum_w_ij = 0.;
            let mut weighted_mean = Vector3::zeros(); // sum of the weighted position of the neighborhood

            for j in snapshot.neighbours(i).iter().chain([i].iter()) {
                let x_j = &snapshot.position(*j);
                let w_ij = self.w((x_i - x_j).norm_squared());

                sum_w_ij += w_ij;
                weighted_mean += w_ij * x_j;
            }

            weighted_mean /= sum_w_ij;

            self.weighted_means[i] = weighted_mean;
            self.smoothed_positions[i] = (1. - self.cst_smoothness) * x_i + self.cst_smoothness * weighted_mean;
        }
    }

    fn compute_covariance_matrix(&self, snapshot: &Box<dyn FluidSnapshot>, i: usize) -> Matrix3<f32> {
        let mut c = Matrix3::zeros();
        let x_i = &snapshot.position(i);
        let x_w_i = &self.weighted_means[i];

        let mut sum_w_ij = 0.;

        for j in snapshot.neighbours(i).iter().chain([i].iter()) {
            let x_j = &snapshot.position(*j);
            let w_ij = self.w((x_i - x_j).norm_squared());

            let variance = x_j - x_w_i;

            sum_w_ij += w_ij;
            c += w_ij * (variance * variance.transpose());
        }

        c / sum_w_ij
    }

    fn normalize_eigen_values(&self, svd: &SVD<f32, U3, U3>) -> Vector3<f32> {
        let (sig_1, sig_2, sig_3) = (
            svd.singular_values[0],
            svd.singular_values[1],
            svd.singular_values[2]
        );

        let sig_min = sig_1 / self.cst_kr;

        Vector3::new(
            self.cst_ks * sig_1,
            self.cst_ks * sig_2.max(sig_min),
            self.cst_ks * sig_3.max(sig_min),
        )
    }

    pub fn smoothed_position(&self, i: usize) -> VertexWorld {
        self.smoothed_positions[i]
    }

    pub fn precompute_positions(&mut self, snapshot: &Box<dyn FluidSnapshot>) {
        self.init_kernels_centers(snapshot);
    }

    pub fn compute_anisotropy(&self, snapshot: &Box<dyn FluidSnapshot>, i: usize) -> Matrix3<f32> {
        let c = self.compute_covariance_matrix(snapshot, i);
        let c_svd = &mut c.svd(true, false);

        let u = c_svd.u.unwrap();

        let eigen_mat_inv = if snapshot.neighbours(i).len() < self.cst_min_nb_neighbours {
            self.cst_kn_inv
        } else {
            let eigen_values = self.normalize_eigen_values(c_svd);
            Matrix3::from_diagonal(&eigen_values).pseudo_inverse(CST_EPSILON).unwrap()
        };

        // println!("self.eigen_val={}", self.cst_kn);
        // println!("eigen_val={}", eigen_mat_inv);
        // println!();

        self.kernel_radius_inv * u * eigen_mat_inv * u.transpose()
    }
}
