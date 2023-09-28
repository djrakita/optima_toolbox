use ad_trait::AD;
use parry_ad::shape::Shape;

pub mod distance;


pub struct ParryShapeWrapper<T: AD> {
    shape: Box<dyn Shape<T>>
}
impl<T: AD> ParryShapeWrapper<T> {
    pub fn new<S: Shape<T>>(shape: S) -> Self {
        Self {
            shape: Box::new(shape),
        }
    }
    pub fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }
}