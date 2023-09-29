use std::any::Any;

pub trait AsAnyTrait {
    fn as_any(&self) -> &dyn Any;
}