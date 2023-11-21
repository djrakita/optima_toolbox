pub mod optimization_engine;

use std::any::Any;
use ad_trait::AD;
use ad_trait::differentiable_block::{DifferentiableBlock, DifferentiableBlockArgPrepTrait};
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};

pub trait DiffBlockUnconstrainedOptimizerTrait {
    type DataType : AD;
    type OutputType : Any + OptimizerOutputTrait<DataType = Self::DataType>;

    fn diff_block_unconstrained_optimize<'a, D1: DifferentiableFunctionTrait, E1: DerivativeMethodTrait, AP: DifferentiableBlockArgPrepTrait<'a, D1, E1>>(&self, objective_function: &DifferentiableBlock<'a, D1, E1, AP>, initial_condition: &[Self::DataType]) -> Self::OutputType;
}

pub trait OptimizerOutputTrait {
    type DataType : AD;

    fn x_star(&self) -> &[Self::DataType];
    fn f_star(&self) -> Self::DataType;
}
impl<O: OptimizerOutputTrait> OptimizerOutputTrait for Box<O> {
    type DataType = O::DataType;

    #[inline(always)]
    fn x_star(&self) -> &[Self::DataType] {
        self.as_ref().x_star()
    }

    #[inline(always)]
    fn f_star(&self) -> Self::DataType {
        self.as_ref().f_star()
    }
}

