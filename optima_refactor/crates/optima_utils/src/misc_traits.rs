use std::fmt::Debug;
use std::hash::Hash;

pub trait IdentifiableTrait {
    type IDType : Clone + Debug + PartialEq + Hash + Default;

    fn id(&self) -> Self::IDType;
}