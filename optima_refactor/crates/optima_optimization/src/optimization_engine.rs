use std::marker::PhantomData;
use std::sync::Mutex;
use ad_trait::differentiable_block::{DifferentiableBlock, DifferentiableBlockArgPrepTrait};
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};
use optimization_engine::core::SolverStatus;
use optimization_engine::panoc::{PANOCCache, PANOCOptimizer};
use optimization_engine::{constraints, Optimizer, Problem, SolverError};
use crate::{DiffBlockUnconstrainedOptimizerTrait, GenericOptimizerTrait, OptimizerOutputTrait};

pub struct SimpleOpEnOptimizer;
impl DiffBlockUnconstrainedOptimizerTrait for SimpleOpEnOptimizer {
    type DataType = f64;
    type ArgsType = SimpleOpEnEngineOptimizerArgs;
    type OutputType = Box<SimpleOpEnEngineOptimizerOutput>;

    fn diff_block_unconstrained_optimize<'a, D, E, AP>(objective_function: &DifferentiableBlock<'a, D, E, AP>, init_condition: &[f64], args: &Self::ArgsType) -> Self::OutputType where D: DifferentiableFunctionTrait, E: DerivativeMethodTrait, AP: DifferentiableBlockArgPrepTrait<'a, D, E> {
        simple_open_optimize(objective_function, init_condition, &args.lower_bounds, &args.upper_bounds, &args.cache)
    }
}

pub struct SimpleOpEnEngineOptimizerArgs {
    lower_bounds: Vec<f64>,
    upper_bounds: Vec<f64>,
    cache: Mutex<PANOCCache>
}
impl SimpleOpEnEngineOptimizerArgs {
    pub fn new(lower_bounds: Vec<f64>, upper_bounds: Vec<f64>, tolerance: f64) -> Self {
        assert_eq!(lower_bounds.len(), upper_bounds.len());
        let problem_size = lower_bounds.len();
        Self { lower_bounds, upper_bounds, cache: Mutex::new(PANOCCache::new(problem_size, tolerance, 3)) }
    }
}

pub struct SimpleOpEnEngineGenericOptimizer<'a, D, E, AP>(PhantomData<(&'a D, E, AP)>) where D: DifferentiableFunctionTrait,
                                                                                             E: DerivativeMethodTrait,
                                                                                             AP: DifferentiableBlockArgPrepTrait<'a, D, E>;
impl<'a, D, E, AP> GenericOptimizerTrait for SimpleOpEnEngineGenericOptimizer<'a, D, E, AP> where D: DifferentiableFunctionTrait,
                                                                                                  E: DerivativeMethodTrait,
                                                                                                  AP: DifferentiableBlockArgPrepTrait<'a, D, E> {
    type DataType = f64;
    type ArgsType = SimpleOpEnEngineGenericOptimizerArgs<'a, D, E, AP>;
    type OutputType = Box<SimpleOpEnEngineOptimizerOutput>;

    fn optimize(args: &Self::ArgsType) -> Self::OutputType {
        simple_open_optimize(&args.objective_function, &args.initial_condition, &args.lower_bounds, &args.upper_bounds, &args.cache)
    }
}

pub struct SimpleOpEnEngineGenericOptimizerArgs<'a, D, E, AP> where D: DifferentiableFunctionTrait,
                                                                    E: DerivativeMethodTrait,
                                                                    AP: DifferentiableBlockArgPrepTrait<'a, D, E> {
    objective_function: DifferentiableBlock<'a, D, E, AP>,
    initial_condition: Vec<f64>,
    lower_bounds: Vec<f64>,
    upper_bounds: Vec<f64>,
    cache: Mutex<PANOCCache>
}
impl<'a, D, E, AP> SimpleOpEnEngineGenericOptimizerArgs<'a, D, E, AP> where D: DifferentiableFunctionTrait,
                                                                            E: DerivativeMethodTrait,
                                                                            AP: DifferentiableBlockArgPrepTrait<'a, D, E> {
    pub fn new(objective_function: DifferentiableBlock<'a, D, E, AP>, initial_condition: Vec<f64>, lower_bounds: Vec<f64>, upper_bounds: Vec<f64>) -> Self {
        let problem_size = initial_condition.len();
        Self { objective_function, initial_condition, lower_bounds, upper_bounds, cache: Mutex::new(PANOCCache::new(problem_size, 0.001, 3)) }
    }
}

fn simple_open_optimize<'a, D, E, AP>(objective_function: &DifferentiableBlock<'a, D, E, AP>, init_condition: &[f64], lower_bounds: &Vec<f64>, upper_bounds: &Vec<f64>, cache: &Mutex<PANOCCache>) -> Box<SimpleOpEnEngineOptimizerOutput>
    where D: DifferentiableFunctionTrait,
          E: DerivativeMethodTrait,
          AP: DifferentiableBlockArgPrepTrait<'a, D, E>
{
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
    let problem = Problem::new(&binding, df, f);
    let mut binding = cache.lock();
    let cache = binding.as_mut().unwrap();
    let mut panoc = PANOCOptimizer::new(problem, cache);

    let mut x = init_condition.to_vec();
    let solver_status = panoc.solve(x.as_mut_slice()).expect("error");

    Box::new(SimpleOpEnEngineOptimizerOutput {
        x_star: x,
        solver_status,
    })
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