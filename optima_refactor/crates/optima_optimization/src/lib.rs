pub mod optimization_engine;

use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};

pub trait GradientBasedOptimizerTrait {
    type OptArgs;
    type OptOutput;

    fn optimize<D, E>(call_args: &D::ArgsType<f64>, grad_args: &D::ArgsType<E::T>, grad_method_data: &E::DerivativeMethodData, opt_args: &mut Self::OptArgs) -> Self::OptOutput
        where D: DifferentiableFunctionTrait,
              E: DerivativeMethodTrait;
}
