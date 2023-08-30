use std::marker::PhantomData;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};
use optimization_engine::core::SolverStatus;
use optimization_engine::panoc::{PANOCCache, PANOCOptimizer};
use optimization_engine::{constraints, Optimizer, Problem, SolverError};
use optima_linalg::vecs_and_mats::OVec;
use crate::GradientBasedOptimizerTrait;


pub struct OpEnSimpleArgs<V: OVec<f64>> {
    pub cache: PANOCCache,
    pub state: V
}

pub struct OpEnSimple<V: OVec<f64>> { _phantom_data: PhantomData<V> }
impl<V: OVec<f64>> GradientBasedOptimizerTrait for OpEnSimple<V> {
    type OptArgs = OpEnSimpleArgs<V>;
    type OptOutput = SolverStatus;

    fn optimize<D, E>(call_args: &D::ArgsType<f64>, grad_args: &D::ArgsType<E::T>, grad_method_data: &E::DerivativeMethodData, opt_args: &mut Self::OptArgs) -> Self::OptOutput where D: DifferentiableFunctionTrait, E: DerivativeMethodTrait {
        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            let res = D::derivative::<E>(u, grad_args, grad_method_data);
            let grad_as_slice = res.1.as_slice();
            assert_eq!(grad_as_slice.len(), grad.len());
            grad.iter_mut().zip(grad_as_slice.iter()).for_each(|(x, y)| *x = *y );
            Ok(())
        };
        let f = |u: &[f64], cost: &mut f64| -> Result<(), SolverError> {
            let res = D::call(u, call_args);
            assert_eq!(res.len(), 1);
            *cost = res[0];
            Ok(())
        };

        let binding = constraints::NoConstraints::new();
        let problem = Problem::new(&binding, df, f);
        let mut panoc = PANOCOptimizer::new(problem, &mut opt_args.cache);

        return panoc.solve(opt_args.state.as_mut_slice_ovec()).expect("error");
    }
}