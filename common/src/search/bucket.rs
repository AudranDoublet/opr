use crate::search::AABB;

#[derive(Clone, Copy)]
pub struct Bucket {
    pub size: usize,
    pub aabb: AABB,
}

impl Bucket {
    pub fn new() -> Bucket {
        Bucket {
            size: 0,
            aabb: AABB::empty(),
        }
    }

    pub fn add_aabb(&mut self, aabb: &AABB) {
        self.size += 1;
        self.aabb = self.aabb.join(aabb);
    }

    pub fn cost(&self) -> f32 {
        self.size as f32 * self.aabb.surface()
    }

    pub fn join(a: Bucket, b: &Bucket) -> Bucket {
        Bucket {
            size: a.size + b.size,
            aabb: a.aabb.join(&b.aabb),
        }
    }

    pub fn join_all(buckets: &[Bucket]) -> Bucket {
        buckets.iter().fold(Bucket::new(), Bucket::join)
    }
}
