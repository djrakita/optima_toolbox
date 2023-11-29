pub mod optimization_engine;
pub mod loss_functions;
use std::any::Any;
use ad_trait::AD;
use ad_trait::differentiable_block::{DifferentiableBlock, DifferentiableBlockArgPrepTrait};
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};

pub trait GenericOptimizerTrait {
    type DataType : AD;
    type ArgsType;
    type OutputType : Any + OptimizerOutputTrait<DataType = Self::DataType>;

    fn optimize(args: &Self::ArgsType) -> Self::OutputType;
}

pub trait DiffBlockUnconstrainedOptimizerTrait {
    type DataType : AD;
    type ArgsType;
    type OutputType : Any + OptimizerOutputTrait<DataType = Self::DataType>;

    fn diff_block_unconstrained_optimize<'a, D, E, AP>(objective_function: &DifferentiableBlock<'a, D, E, AP>, init_condition: &[f64], args: &Self::ArgsType) -> Self::OutputType
        where D: DifferentiableFunctionTrait,
              E: DerivativeMethodTrait,
              AP: DifferentiableBlockArgPrepTrait<'a, D, E>;
}

pub struct OwnedGenericOptimizer<O: GenericOptimizerTrait> {
    args: O::ArgsType
}
impl<O: GenericOptimizerTrait> OwnedGenericOptimizer<O> where O: GenericOptimizerTrait {
    pub fn new(args: O::ArgsType) -> Self {
        Self { args }
    }
    pub fn optimize(&self) -> O::OutputType {
        O::optimize(&self.args)
    }
    #[inline(always)]
    pub fn args_ref(&self) -> &O::ArgsType {
        &self.args
    }
    #[inline(always)]
    pub fn args_mut(&mut self) -> &mut O::ArgsType {
        &mut self.args
    }
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
