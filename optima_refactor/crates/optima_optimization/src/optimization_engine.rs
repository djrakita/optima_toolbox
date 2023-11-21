use std::sync::Mutex;
use ad_trait::differentiable_block::{DifferentiableBlock, DifferentiableBlockArgPrepTrait};
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};
use optimization_engine::core::SolverStatus;
use optimization_engine::panoc::{PANOCCache, PANOCOptimizer};
use optimization_engine::{constraints, Optimizer, Problem, SolverError};
use crate::{DiffBlockUnconstrainedOptimizerTrait, OptimizerOutputTrait};

pub struct SimpleOpEnEngineOptimizer {
    problem_size: usize,
    pub lower_bounds: Vec<f64>,
    pub upper_bounds: Vec<f64>,
    pub cache: Mutex<PANOCCache>
}
impl SimpleOpEnEngineOptimizer {
    pub fn new(lower_bounds: Vec<f64>, upper_bounds: Vec<f64>, tolerance: f64) -> Self {
        assert_eq!(lower_bounds.len(), upper_bounds.len());
        let problem_size = lower_bounds.len();

        Self {
            problem_size,
            lower_bounds,
            upper_bounds,
            cache: Mutex::new(PANOCCache::new(problem_size, tolerance, 3)),
        }
    }
}
impl DiffBlockUnconstrainedOptimizerTrait for SimpleOpEnEngineOptimizer {
    type DataType = f64;
    type OutputType = Box<SimpleOpEnEngineOptimizerOutput>;

    fn diff_block_unconstrained_optimize<'a, D1: DifferentiableFunctionTrait, E1: DerivativeMethodTrait, AP: DifferentiableBlockArgPrepTrait<'a, D1, E1>>(&self, objective_function: &DifferentiableBlock<'a, D1, E1, AP>, initial_condition: &[f64]) -> Self::OutputType {
        assert_eq!(self.problem_size, initial_condition.len());

        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            let res = objective_function.derivative(u);
            let grad_as_slice = res.1.as_slice();
            assert_eq!(grad_as_slice.len(), grad.len());
            grad.iter_mut().zip(grad_as_slice.iter()).for_each(|(x, y)| *x = *y );
            Ok(())
        };
        let f = |u: &[f64], cost: &mut f64| -> Result<(), SolverError> {
            let res=  objective_function.call(u);
            assert_eq!(res.len(), 1);
            *cost = res[0];
            Ok(())
        };

        let binding = constraints::Rectangle::new(Some(&self.lower_bounds), Some(&self.upper_bounds));
        let problem = Problem::new(&binding, df, f);
        let mut binding = self.cache.lock();
        let cache = binding.as_mut().unwrap();
        let mut panoc = PANOCOptimizer::new(problem, cache);

        let mut x = initial_condition.to_vec();
        let solver_status = panoc.solve(x.as_mut_slice()).expect("error");

        Box::new(SimpleOpEnEngineOptimizerOutput {
            x_star: x,
            solver_status,
        })
    }
}

pub struct SimpleOpEnEngineOptimizerOutput {
    x_star: Vec<f64>,
    solver_status: SolverStatus
}
impl SimpleOpEnEngineOptimizerOutput {
    #[inline(always)]
    pub fn solver_status(&self) -> SolverStatus {
        self.solver_status
    }
}
impl OptimizerOutputTrait for SimpleOpEnEngineOptimizerOutput {
    type DataType = f64;

    #[inline(always)]
    fn x_star(&self) -> &[Self::DataType] {
        self.x_star.as_slice()
    }

    #[inline(always)]
    fn f_star(&self) -> Self::DataType {
        self.solver_status.cost_value()
    }
}