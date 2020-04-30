use nalgebra::{Matrix3, SVD, U3, Vector3};

use crate::types::{FluidSnapshot, VertexWorld};

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

    cst_smoothness: f32, // λ ~= 0.9

    cst_min_nb_neighbours: usize,

    // k_r > 1. used to prevent extreme deformation (σ_k = max(σ_k, σ_1/k_r))
    cst_kr: f32,

    // k_s such that ||k_s * C|| ~= 1 => to avoid ||C|| explosion when the neighborhood is full.
    // Considering the fact that the covariance matrix C contains small values of the order of the cube of the smoothing radius h^3),
    // so a correct value k_s should be of the order of h^(-3) (ex: 1400 in the paper) so ||k_s * C|| ~= 1
    cst_ks: f32,

    // k_n such that, on too low neighborhood, Σ~ = k_n * I. Should be similar to the kernel radius
    cst_kn: f32,
}

impl Anisotropicator {
    pub fn new(cst_smoothness: f32, cst_min_nb_neighbours: usize, cst_kr: f32, cst_ks: f32, cst_kn: f32) -> Anisotropicator {
        assert!(0. <= cst_smoothness && cst_smoothness <= 1.);

        Anisotropicator {
            smoothed_positions: vec![],
            weighted_means: vec![],

            kernel_radius_inv: -1.,
            radius: -1.,

            cst_smoothness,
            cst_min_nb_neighbours,
            cst_kr,
            cst_ks,
            cst_kn,
        }
    }

    /// isotropic weighting function 2 particles at positions x_i and x_j
    /// * `n_sq`: ||x_j - x_i||^2
    fn w(&self, d: f32) -> f32 {
        if d < self.radius {
            1. - (d / self.radius).powi(3)
        } else {
            0.
        }
    }

    pub fn compute_radius(kernel_radius: f32) -> f32 {
        2. * kernel_radius
    }

    fn init_kernels_centers(&mut self, snapshot: &impl FluidSnapshot) {
        let kernel = snapshot.get_kernel();

        self.kernel_radius_inv = 1. / kernel.radius();
        self.radius = Anisotropicator::compute_radius(kernel.radius());

        self.smoothed_positions.resize(snapshot.len(), Vector3::zeros());
        self.weighted_means.resize(snapshot.len(), Vector3::zeros());

        for i in 0..snapshot.len() {
            let x_i = &snapshot.position(i);

            let mut sum_w_ij = 1.;
            let mut weighted_mean = *x_i;

            for j in snapshot.neighbours_anisotropic_kernel(i) {
                let x_j = &snapshot.position(*j);
                let w_ij = self.w((x_i - x_j).norm());

                sum_w_ij += w_ij;
                weighted_mean += w_ij * x_j;
            }

            weighted_mean /= sum_w_ij;

            self.weighted_means[i] = weighted_mean;
            self.smoothed_positions[i] = (1. - self.cst_smoothness) * x_i + self.cst_smoothness * weighted_mean;
        }
    }

    fn compute_covariance_matrix(&self, snapshot: &impl FluidSnapshot, i: usize) -> Matrix3<f32> {
        let mut c = Matrix3::zeros();
        let x_w_i = &self.weighted_means[i];

        let mut sum_w_ij = 1.;

        for j in snapshot.neighbours_anisotropic_kernel(i).iter() {
            let x_j = &snapshot.position(*j);
            let variance = x_j - x_w_i;
            let w_ij = self.w(variance.norm());
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

        let sig_min = svd.singular_values.max() / self.cst_kr;

        self.cst_ks * Vector3::new(
            sig_1.max(sig_min),
            sig_2.max(sig_min),
            sig_3.max(sig_min),
        )
    }

    pub fn smoothed_position(&self, i: usize) -> VertexWorld {
        self.smoothed_positions[i]
    }

    pub fn precompute_positions(&mut self, snapshot: &impl FluidSnapshot) {
        self.init_kernels_centers(snapshot);
    }

    pub fn compute_anisotropy(&self, snapshot: &impl FluidSnapshot, i: usize) -> Matrix3<f32> {
        self.kernel_radius_inv * if snapshot.neighbours_anisotropic_kernel(i).len() < self.cst_min_nb_neighbours {
            Matrix3::identity() * (1. / self.cst_kn)
        } else {
            let c = self.compute_covariance_matrix(snapshot, i);
            let c_svd = &mut c.svd(true, false);

            let u = c_svd.u.unwrap();

            let eigen_mat_inv = {
                let eigen_values_inv = self.normalize_eigen_values(c_svd).apply_into(|v| 1. / v);
                Matrix3::from_diagonal(&eigen_values_inv)
            };

            u * eigen_mat_inv * u.transpose()
        }
    }
}
