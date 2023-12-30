use ad_trait::AD;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DifferentiableFunctionClass, DifferentiableFunctionTrait, FiniteDifferencing, ForwardAD, ReverseAD};
use optima_optimization::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization::nlopt::{Algorithm, NLOptOptimizer};

pub struct TestClass;
impl DifferentiableFunctionClass for TestClass {
    type FunctionType<'a, T: AD> = Test;
}

pub struct Test;
impl<'a, T: AD> DifferentiableFunctionTrait<'a, T> for Test {
    fn call(&self, inputs: &[T], _freeze: bool) -> Vec<T> {
        vec![inputs[0].powi(2)]
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        1
    }
}

fn main() {
    let db = DifferentiableBlock::new_with_tag(TestClass, ForwardAD::new(), Test, Test);
    let f = db.derivative(&[1.0]);
    println!("{:?}", f);

    let o = NLOptOptimizer::new(Algorithm::Ccsaq, false, 1, Some(10), None, None);
    let res = o.optimize_unconstrained(&[2.0], &db);
    println!("{:?}", res);
}