use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};
use optimization_engine::core::SolverStatus;
use optimization_engine::panoc::{PANOCCache, PANOCOptimizer};
use optimization_engine::{constraints, Optimizer, Problem, SolverError};
use optima_linalg::OVec;
use crate::derivative_based_optimization::DerivBasedOptTrait;

pub struct OpEnSimpleArgs {
    pub cache: PANOCCache,
    pub state: Vec<f64>
}
impl OpEnSimpleArgs {
    pub fn new(cache: PANOCCache, state: Vec<f64>) -> Self {
        Self {
            cache,
            state,
        }
    }
}

#[derive(Clone, Debug)]
pub struct OpEnSimpleOutput {
    x_opt: Vec<f64>,
    solver_status: SolverStatus
}
impl OpEnSimpleOutput {
    pub fn x_opt(&self) -> &Vec<f64> {
        &self.x_opt
    }
    pub fn solver_status(&self) -> SolverStatus {
        self.solver_status
    }
}

pub struct OpEnSimple;
impl DerivBasedOptTrait for OpEnSimple {
    type OptOutput = OpEnSimpleOutput;
    type OptArgs = OpEnSimpleArgs;

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

        let solver_status = panoc.solve(opt_args.state.as_mut_slice_ovec()).expect("error");

        OpEnSimpleOutput {
            x_opt: opt_args.state.clone(),
            solver_status,
        }
    }
}