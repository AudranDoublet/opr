extern crate kiss3d;

pub use kiss3d::event;
pub mod particle;
pub mod scene;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
