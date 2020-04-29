pub enum ConvMatrix
{
    Basic {
        values: Vec<f32>,
        size: usize,
        div: f32,
    },
    Combine {
        a: Box<ConvMatrix>,
        b: Box<ConvMatrix>,
        size: usize,
    },
    Adapative {
        values: Vec<f32>,
        size: usize,
        adapt: Box<dyn Fn(f32, f32) -> f32 + Sync>,
    },
}

impl ConvMatrix
{
    pub fn new_3x3(values: Vec<f32>) -> ConvMatrix {
        if values.len() != 9 {
            panic!("bad 3x3 convolution matrix: size should be 9");
        }

        ConvMatrix::Basic {
            values: values,
            size: 9,
            div: 1.,
        }
    }

    pub fn new_combined_3x3(a: ConvMatrix, b: ConvMatrix) -> ConvMatrix {
        if b.size() != 9 || a.size() != 9 {
            panic!("bad 3x3 convolution matrix: size should be 9");
        }

        ConvMatrix::Combine {
            a: Box::new(a),
            b: Box::new(b),
            size: 9,
        }
    }

    pub fn new_adapative_3x3(values: Vec<f32>, f: Box<dyn Fn(f32, f32) -> f32 + Sync>) -> ConvMatrix {
        if values.len() != 9 {
            panic!("bad 3x3 convolution matrix: size should be 9");
        }

        ConvMatrix::Adapative {
            values: values,
            size: 9,
            adapt: f,
        }
    }

    pub fn laplacian() -> ConvMatrix {
        ConvMatrix::new_3x3(vec![
            0., -1., 0.,
            -1., 4., -1.,
            0., -1., 0.,
        ])
    }

    pub fn laplacian_augment(k: f32) -> ConvMatrix {
        ConvMatrix::new_3x3(vec![
            0.       , -1. * k      , 0.,
            -1. * k  , 4.  * k + 1. , -1. * k,
            0.       , -1. * k      , 0.,
        ])
    }

    pub fn gaussian_augment(k: f32) -> ConvMatrix {
        let k = k / 16.;

        ConvMatrix::new_3x3(vec![
            -1. * k   , -2. * k     , -1. * k,
            -2. * k   , -4. * k + 2., -2. * k,
            -1. * k   , -2. * k     , -1. * k,
        ])
    }

    pub fn gaussian_adaptative_lazy(threshold: f32) -> ConvMatrix {
        ConvMatrix::new_adapative_3x3(vec![
            1.    , 2.      , 1. ,
            2.    , 4.      , 2. ,
            1.    , 2.      , 1. ,
        ], Box::new(move |c, color| match (color - c).abs() {
            v if v <= threshold => 1.0,
            _ => 0.0,
        }))
    }

    pub fn gaussian_adaptative_smart(coeff: f32) -> ConvMatrix {
        ConvMatrix::new_adapative_3x3(vec![
            1.    , 2.      , 1. ,
            2.    , 4.      , 2. ,
            1.    , 2.      , 1. ,
        ], Box::new(move |c, color| (255. - color + c).abs() * coeff / 255.))
    }

    pub fn size(&self) -> usize {
        match self {
            ConvMatrix::Basic { size, .. } => *size,
            ConvMatrix::Adapative { size, .. } => *size,
            ConvMatrix::Combine { size, .. } => *size,
        }
    }

    pub fn apply(&self, slice: &Vec<f32>) -> f32 {
        if slice.len() > self.size() {
            println!("{:?} {:?}", slice.len(), self.size());
            panic!("can't apply convolution: kernel size should be smaller or equal to image slice");
        }

        match self {
            ConvMatrix::Basic { values, div, .. } => {
                let mut sum = 0.;

                for v in 0..slice.len() {
                    sum += values[v] * slice[v];
                }

                sum / *div
            },
            ConvMatrix::Adapative { values, adapt, .. } => {
                let center_val = slice[slice.len() / 2 + 1];
                let (mut div, mut sum) = (0., 0.);

                for v in 0..slice.len() {
                    let coeff = (adapt)(center_val, slice[v]) * values[v];

                    sum += slice[v] * coeff;
                    div += coeff;
                }

                sum / div
            },
            ConvMatrix::Combine { a, b, .. } => {
                let va = a.apply(slice);
                let vb = b.apply(slice);

                (va*va + vb*vb).sqrt()
            },
        }
    }
}
