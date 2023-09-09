use ad_trait::AD;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait, ForwardAD};
use optimization_engine::panoc::PANOCCache;
use optima_optimization::derivative_based_optimization::{DerivBasedOptSolver};
use optima_optimization::derivative_based_optimization::optimization_engine::{OpEnSimple, OpEnSimpleArgs};

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
    let opt_args = OpEnSimpleArgs::new(PANOCCache::new(2, 1e-5, 3), [1.,2.]);
    let mut solver = DerivBasedOptSolver::<Test, ForwardAD, OpEnSimple<_>>::new((), (), (), opt_args);

    solver.opt_args_mut().state = [300., 4.];

    let res = solver.optimize();
    println!("{:?}", res);
}