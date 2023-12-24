pub mod open;
pub mod argmin;
pub mod loss_functions;

use std::any::Any;
use ad_trait::AD;
use ad_trait::differentiable_block::{DifferentiableBlock, DifferentiableBlockZero};
use ad_trait::differentiable_function::{DerivativeAlwaysZero, DerivativeMethodTrait, DifferentiableFunctionClass};

pub trait DiffBlockOptimizerTrait {
    type OutputType : Any + OptimizerOutputTrait<DataType = f64>;

    fn optimize<'a, DC1, E1, DC2, E2, DC3, E3>(&self, initial_condition: &[f64], objective_function: &DifferentiableBlock<'a, DC1, E1>, equality_constraint_function: &DifferentiableBlock<'a, DC2, E2>, inequality_constraint_function: &DifferentiableBlock<'a, DC3, E3>) -> Self::OutputType
        where DC1: DifferentiableFunctionClass,
              DC2: DifferentiableFunctionClass,
              DC3: DifferentiableFunctionClass,
              E1: DerivativeMethodTrait,
              E2: DerivativeMethodTrait,
              E3: DerivativeMethodTrait;

    fn optimize_unconstrained<'a, DC1, E1>(&self, initial_condition: &[f64], objective_function: &DifferentiableBlock<'a, DC1, E1>) -> Self::OutputType
        where DC1: DifferentiableFunctionClass, E1: DerivativeMethodTrait
    {
        let c = DifferentiableBlockZero::new_zero(DerivativeAlwaysZero, objective_function.num_inputs(), objective_function.num_outputs());
        self.optimize(initial_condition, objective_function, &c, &c)
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
