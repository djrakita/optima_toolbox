use ad_trait::AD;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait, ForwardAD, ReverseAD};
use optima_optimization2::{DiffBlockUnconstrainedOptimizerTrait, OptimizerOutputTrait};
use optima_optimization2::optimization_engine::SimpleOpEnEngineOptimizer;

pub struct D;
impl DifferentiableFunctionTrait for D {
    type ArgsType<'a, T: AD> = ();

    fn call<'a, T1: AD>(inputs: &[T1], _args: &Self::ArgsType<'a, T1>) -> Vec<T1> {
        vec![inputs[0].sin()]
    }

    fn num_inputs<T1: AD>(_args: &Self::ArgsType<'_, T1>) -> usize {
        1
    }

    fn num_outputs<T1: AD>(_args: &Self::ArgsType<'_, T1>) -> usize {
        1
    }
}


fn main() {
    let o = SimpleOpEnEngineOptimizer::new(vec![-100.0], vec![100.0], 0.001);

    let d = DifferentiableBlock::<D, ReverseAD>::new((), (), ());

    let res = o.diff_block_unconstrained_optimize(&d, &vec![-1.0]);
    println!("{:?}", res.x_star());
    println!("{:?}", res.f_star());
}