use std::sync::Mutex;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionClass};
use optimization_engine::core::SolverStatus;
use optimization_engine::panoc::{PANOCCache, PANOCOptimizer};
use optimization_engine::{constraints, Optimizer, Problem, SolverError};
use optima_linalg::OVec;
use optima_sampling::SimpleSampler;
use crate::{DiffBlockOptimizerTrait, OptimizerOutputTrait};

pub struct SimpleOpEnOptimizer {
    lower_bounds: Vec<f64>,
    upper_bounds: Vec<f64>,
    panoc_cache: Mutex<PANOCCache>,
}
impl SimpleOpEnOptimizer {
    pub fn new(lower_bounds: Vec<f64>, upper_bounds: Vec<f64>, tolerance: f64) -> Self {
        assert_eq!(lower_bounds.len(), upper_bounds.len());
        let problem_size = lower_bounds.len();
        Self { lower_bounds, upper_bounds, panoc_cache: Mutex::new(PANOCCache::new(problem_size, tolerance, 5)) }
    }
}
impl DiffBlockOptimizerTrait for SimpleOpEnOptimizer {
    type OutputType = Box<SimpleOpEnEngineOptimizerOutput>;

    fn optimize<'a, DC1, E1, DC2, E2, DC3, E3>(&self, initial_condition: &[f64], objective_function: &DifferentiableBlock<'a, DC1, E1>, _equality_constraint_function: &DifferentiableBlock<'a, DC2, E2>, _inequality_constraint_function: &DifferentiableBlock<'a, DC3, E3>) -> Self::OutputType where DC1: DifferentiableFunctionClass, DC2: DifferentiableFunctionClass, DC3: DifferentiableFunctionClass, E1: DerivativeMethodTrait, E2: DerivativeMethodTrait, E3: DerivativeMethodTrait {
        simple_open_optimize(objective_function, initial_condition, &self.lower_bounds, &self.upper_bounds, &self.panoc_cache)
    }
}

fn simple_open_optimize<'a, DC, E>(objective_function: &DifferentiableBlock<'a, DC, E>, init_condition: &[f64], lower_bounds: &Vec<f64>, upper_bounds: &Vec<f64>, cache: &Mutex<PANOCCache>) -> Box<SimpleOpEnEngineOptimizerOutput> where DC: DifferentiableFunctionClass, E: DerivativeMethodTrait {
    let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        let res = objective_function.derivative(u);
        let grad_as_slice = res.1.as_slice();
        assert_eq!(grad_as_slice.len(), grad.len());
        grad.iter_mut().zip(grad_as_slice.iter()).for_each(|(x, y)| *x = *y );
        Ok(())
    };
    let f = |u: &[f64], cost: &mut f64| -> Result<(), SolverError> {
        let res= objective_function.call(u);
        assert_eq!(res.len(), 1);
        *cost = res[0];
        Ok(())
    };

    let binding = constraints::Rectangle::new(Some(lower_bounds), Some(upper_bounds));
    // let binding = constraints::NoConstraints::new();
    let problem = Problem::new(&binding, df, f);
    let mut binding = cache.lock();
    let cache = binding.as_mut().unwrap();
    let mut panoc = PANOCOptimizer::new(problem, cache);
    // panoc = panoc.with_max_iter(3);

    let s = SimpleSampler::uniform_samples(&vec![(-0.000001, 0.000001); init_condition.len()], None);
    let mut x = init_condition.to_vec().ovec_add(&s);
    // let mut x = init_condition.to_vec();
    let solver_status = panoc.solve(x.as_mut_slice()).expect("error");

    Box::new(SimpleOpEnEngineOptimizerOutput {
        x_star: x,
        solver_status,
    })
}

#[derive(Clone, Debug)]
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