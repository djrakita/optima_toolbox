use std::time::Instant;
use ad_trait::AD;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait, FiniteDifferencing, ForwardAD, ForwardADMulti, ReverseAD};
use ad_trait::forward_ad::adf::{adf_f32x16, adf_f32x2, adf_f32x4};
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::DVector;
use optimization_engine::panoc::PANOCCache;
use optima_optimization::GradientBasedOptimizerTrait;
use optima_optimization::optimization_engine::{OpEnSimple, OpEnSimpleArgs};

pub struct Test;
impl DifferentiableFunctionTrait for Test {
    type ArgsType<T: AD> = ();

    fn call<T1: AD>(inputs: &[T1], _args: &Self::ArgsType<T1>) -> Vec<T1> {
        let output = (inputs[0] * inputs[1]).sin();

        vec![ output ]
    }

    fn num_inputs<T1: AD>(_args: &Self::ArgsType<T1>) -> usize {
        2
    }

    fn num_outputs<T1: AD>(_args: &Self::ArgsType<T1>) -> usize {
        1
    }
}

fn main() {
    let mut a = OpEnSimpleArgs {
        cache: PANOCCache::new(2, 1e-5, 3),
        state: vec![3., 3.],
    };

    let res = OpEnSimple::optimize::<Test, FiniteDifferencing>(&(), &(), &(), &mut a);
    println!("{:?}", res);

    let mut a = OpEnSimpleArgs {
        cache: PANOCCache::new(2, 1e-5, 3),
        state: vec![3., 4.],
    };

    let res = OpEnSimple::optimize::<Test, ForwardADMulti<adf_f32x2>>(&(), &(), &(), &mut a);
    println!("{:?}", res);

    let mut a = OpEnSimpleArgs {
        cache: PANOCCache::new(2, 1e-5, 3),
        state: vec![2., 4.],
    };

    let res = OpEnSimple::optimize::<Test, ForwardADMulti<adf_f32x2>>(&(), &(), &(), &mut a);
    println!("{:?}", res);
    println!("{:?}", a.state);
}