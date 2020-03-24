use nalgebra::{Dynamic, Matrix, VecStorage, Vector3};

pub type DMatrixf32 = Matrix<f32, Dynamic, Dynamic, VecStorage<f32, Dynamic, Dynamic>>;

/*
 * Implementation based on https://github.com/DomiDre/gauss-quad/blob/master/src/legendre/mod.rs
 * (but adapted to dim 3)
 */

pub struct GaussLegendre {
    pub nodes: Vec<f32>,
    pub weights: Vec<f32>,
}

impl GaussLegendre {
    pub fn init(deg: usize) -> GaussLegendre {
        let (nodes, weights) = GaussLegendre::nodes_and_weights(deg);

        GaussLegendre { nodes, weights }
    }

    /// Apply Golub-Welsch algorithm to determine Gauss-Legendre nodes & weights
    /// construct companion matrix A for the Hermite Polynomial using the relation:
    /// (n+1)/(2n+1) P_{n+1} + n/(2n+1) P_{n-1} = x P_n
    /// A similar matrix that is symmetrized is constructed by C = D A D^{-1}
    /// where D is a diagonal matrix and det(C-t*1) = det(A-t*1)
    /// Resulting in a symmetric tridiagonal matrix with
    /// 0 on the diagonal & n/sqrt(4n^2 - 1) on the off-diagonal.
    /// Root & weight finding are equivalent to eigenvalue problem.
    /// see Gil, Segura, Temme - Numerical Methods for Special Functions
    pub fn nodes_and_weights(deg: usize) -> (Vec<f32>, Vec<f32>) {
        let mut companion_matrix = DMatrixf32::from_element(deg, deg, 0.0);
        // Initialize symmetric companion matrix
        for idx in 0..deg - 1 {
            let idx_f32 = 1.0 + idx as f32;
            let element = idx_f32 / (4.0 * idx_f32 * idx_f32 - 1.0).sqrt();
            unsafe {
                *companion_matrix.get_unchecked_mut((idx, idx + 1)) = element;
                *companion_matrix.get_unchecked_mut((idx + 1, idx)) = element;
            }
        }
        // calculate eigenvalues & vectors
        let eigen = companion_matrix.symmetric_eigen();

        // return nodes and weights as Vec<f32>
        let nodes = eigen.eigenvalues.data.as_vec().clone();
        let weights = (eigen.eigenvectors.row(0).map(|x| x.powi(2)) * 2.0)
            .data
            .as_vec()
            .clone();

        (nodes, weights)
    }

    fn argument_transformation(x: f32, a: f32, b: f32) -> f32 {
        0.5 * ((b - a) * x + (b + a))
    }

    fn scale_factor(a: Vector3<f32>, b: Vector3<f32>) -> f32 {
        let vec = 0.5 * (b - a);

        vec.x * vec.y * vec.z
    }

    /// Perform quadrature of integrand using given nodes x and weights w
    pub fn integrate<F>(&self, a: Vector3<f32>, b: Vector3<f32>, integrand: F) -> f32
    where
        F: Fn(Vector3<f32>) -> f32,
    {
        let mut sum = 0.0;

        for i in 0..self.nodes.len() {
            for j in 0..self.nodes.len() {
                for k in 0..self.nodes.len() {
                    let pos = Vector3::new(
                        GaussLegendre::argument_transformation(self.nodes[i], a.x, b.x),
                        GaussLegendre::argument_transformation(self.nodes[j], a.y, b.y),
                        GaussLegendre::argument_transformation(self.nodes[k], a.z, b.z),
                    );

                    sum += self.weights[i] * self.weights[j] * self.weights[k] * integrand(pos);
                }
            }
        }

        GaussLegendre::scale_factor(a, b) * sum
    }
}
