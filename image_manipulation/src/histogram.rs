pub struct Histogram
{
    data: Vec<usize>,
    total: usize,
}

impl Histogram {
    pub fn new() -> Histogram
    {
        Histogram {
            data: vec![0; 256],
            total: 0,
        }
    }

    pub fn add(&mut self, value: u16)
    {
        self.data[value as usize] += 1;
        self.total += 1;
    }

    pub fn get_total(&self) -> usize
    {
        self.total
    }

    pub fn threshold(&self, threshold: usize) -> (u16, u16)
    {
        let mut count = 0;
        let (mut min, mut max) = (0, 0);

        for i in 0..256 {
            count += self.data[i]; 

            if count >= threshold {
                min = i as u16;
                break;
            }
        }

        count = 0;

        for i in (0..256).rev() {
            count += self.data[i]; 

            if count >= threshold {
                max = i as u16;
                break;
            }
        }

        (min, max)
    }

    pub fn cumulative(&self) -> Histogram {
        let mut cum = Histogram::new();

        cum.data[0] = self.data[0];

        for i in 1..256 {
            cum.data[i] = cum.data[i - 1] + self.data[i];
        }

        cum.total = self.total;

        cum
    }

    pub fn at(&self, i: u16) -> usize {
        self.data[i as usize]
    }
}
