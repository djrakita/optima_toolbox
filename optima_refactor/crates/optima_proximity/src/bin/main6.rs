#![feature(associated_type_bounds)]

use std::fmt::Debug;

pub trait TestTrait {
    type T;
}
pub trait TestTrait2 : TestTrait< T: Debug > {
    fn print(t: &Self::T) {
        println!("{:?}", t);
    }
}

pub struct Test;
impl TestTrait for Test {
    type T = Test2;
}
impl TestTrait2 for Test { }

#[derive(Debug)]
pub struct Test2;

fn main() {
    Test::print(&Test2);
}