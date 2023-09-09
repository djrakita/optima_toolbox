pub mod optimization_engine;

use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};

pub trait DerivBasedOptTrait {
    type OptOutput;
    type OptArgs;

    fn optimize<D, E>(call_args: &D::ArgsType<f64>, grad_args: &D::ArgsType<E::T>, grad_method_data: &E::DerivativeMethodData, opt_args: &mut Self::OptArgs) -> Self::OptOutput
        where D: DifferentiableFunctionTrait,
              E: DerivativeMethodTrait;
}

pub struct DerivBasedOptSolver<D: DifferentiableFunctionTrait, E: DerivativeMethodTrait, O: DerivBasedOptTrait> {
    objective_function: DifferentiableBlock<D, E>,
    opt_args: O::OptArgs
}
impl<D: DifferentiableFunctionTrait, E: DerivativeMethodTrait, O: DerivBasedOptTrait> DerivBasedOptSolver<D, E, O> {
    pub fn new(objective_function_standard_args: D::ArgsType<f64>, objective_function_derivative_args: D::ArgsType<E::T>, derivative_method_data: E::DerivativeMethodData, opt_args: O::OptArgs) -> Self {
        Self {
            objective_function: DifferentiableBlock::new(objective_function_standard_args, objective_function_derivative_args, derivative_method_data),
            opt_args,
        }
    }
    pub fn optimize(&mut self) -> O::OptOutput {
        O::optimize::<D, E>(self.objective_function.function_standard_args(), self.objective_function.function_derivative_args(), self.objective_function.derivative_method_data(), &mut self.opt_args)
    }
    pub fn objective_function(&self) -> &DifferentiableBlock<D, E> {
        &self.objective_function
    }
    pub fn objective_function_mut(&mut self) -> &mut DifferentiableBlock<D, E> {
        &mut self.objective_function
    }
    pub fn opt_args(&self) -> &O::OptArgs {
        &self.opt_args
    }
    pub fn opt_args_mut(&mut self) -> &mut O::OptArgs {
        &mut self.opt_args
    }
}