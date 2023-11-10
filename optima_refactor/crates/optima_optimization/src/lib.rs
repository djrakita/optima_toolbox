pub mod optimization_engine;

use std::any::Any;
use ad_trait::AD;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};

pub trait OptimizerTrait {
    type DataType : AD;
    type OutputType : Any + OptimizerOutputTrait<DataType = Self::DataType>;

    fn optimize(&self) -> Self::OutputType;
}

pub trait OptimizerOutputTrait {
    type DataType : AD;

    fn x_star(&self) -> &[Self::DataType];
    fn f_star(&self) -> Self::DataType;
}

pub trait ADTraitBasedOptimizerTrait<D: DifferentiableFunctionTrait + 'static, E: DerivativeMethodTrait + 'static> : OptimizerTrait {
    fn update_objective_args<U: Fn(&mut D::ArgsType<'_, f64>, &mut D::ArgsType<'_, E::T>)>(&mut self, update_fn: U);
    fn update_initial_condition(&mut self, v: &[Self::DataType]);
}

/*
/// non-differentiable derivative-based AD trait unconstrained opt trait
pub trait NdDbAdtUOptTrait<D: DifferentiableFunctionTrait, E: DerivativeMethodTrait> : for<'a> OptTrait< ArgsType<'a> = NdDbAdtUOptArgs<'a, D, E> > { }

/// non-differentiable derivative-based AD trait unconstrained args
pub struct NdDbAdtUOptArgs<'a, D: DifferentiableFunctionTrait, E: DerivativeMethodTrait> {
    initial_condition: &'a [f64],
    objective_function: &'a DifferentiableBlock<D, E>
}
*/



