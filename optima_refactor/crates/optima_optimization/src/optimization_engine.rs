use std::sync::Mutex;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};
use optimization_engine::core::SolverStatus;
pub use optimization_engine::panoc::{PANOCCache, PANOCOptimizer};
pub use optimization_engine::{constraints, Optimizer, Problem, SolverError};
use crate::{OptimizerOutputTrait, OptimizerDiffBlockObjectiveFunctionTrait, OptimizerTrait, OptimizerInitialConditionTrait, DiffBlockObjectiveOptimizerConstructorTrait, OptimizerBoundsTrait};

pub struct SimpleOpEnEngineOptimizer<'a, D: DifferentiableFunctionTrait + 'static, E: DerivativeMethodTrait + 'static> {
    pub differentiable_block: DifferentiableBlock<'a, D, E>,
    pub initial_condition: Vec<f64>,
    pub lower_bounds: Vec<f64>,
    pub upper_bounds: Vec<f64>,
    pub cache: Mutex<PANOCCache>
}
impl<'a, D: DifferentiableFunctionTrait, E: DerivativeMethodTrait> OptimizerTrait for SimpleOpEnEngineOptimizer<'a, D, E> {
    type DataType = f64;
    type OutputType = Box<SimpleOpEnEngineOptimizerOutput>;

    fn optimize(&self) -> Self::OutputType {
        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            let res = self.differentiable_block.derivative(u);
            let grad_as_slice = res.1.as_slice();
            assert_eq!(grad_as_slice.len(), grad.len());
            grad.iter_mut().zip(grad_as_slice.iter()).for_each(|(x, y)| *x = *y );
            Ok(())
        };
        let f = |u: &[f64], cost: &mut f64| -> Result<(), SolverError> {
            let res=  self.differentiable_block.call(u);
            assert_eq!(res.len(), 1);
            *cost = res[0];
            Ok(())
        };

        let binding = constraints::Rectangle::new(Some(&self.lower_bounds), Some(&self.upper_bounds));
        let problem = Problem::new(&binding, df, f);
        let mut binding = self.cache.lock();
        let cache = binding.as_mut().unwrap();
        let mut panoc = PANOCOptimizer::new(problem, cache);

        let mut x = self.initial_condition.to_vec();
        let solver_status = panoc.solve(x.as_mut_slice()).expect("error");
        
        Box::new(SimpleOpEnEngineOptimizerOutput {
            x_star: x,
            solver_status,
        })
    }
}
impl<'a, D: DifferentiableFunctionTrait, E: DerivativeMethodTrait> OptimizerDiffBlockObjectiveFunctionTrait<'a> for SimpleOpEnEngineOptimizer<'a, D, E>  {
    type D = D;
    type E = E;

    #[inline(always)]
    fn objective_function_differentiable_block(&self) -> &DifferentiableBlock<'a, Self::D, Self::E> {
        &self.differentiable_block
    }

    #[inline(always)]
    fn objective_function_differentiable_block_mut(&mut self) -> &mut DifferentiableBlock<'a, Self::D, Self::E> {
        &mut self.differentiable_block
    }
}
impl<'a, D: DifferentiableFunctionTrait, E: DerivativeMethodTrait> OptimizerInitialConditionTrait for SimpleOpEnEngineOptimizer<'a, D, E> {
    fn initial_condition_ref(&self) -> &Vec<Self::DataType> {
        &self.initial_condition
    }

    fn initial_condition_mut(&mut self) -> &mut Vec<Self::DataType> {
        &mut self.initial_condition
    }
}
impl<'a, D: DifferentiableFunctionTrait, E: DerivativeMethodTrait> OptimizerBoundsTrait for SimpleOpEnEngineOptimizer<'a, D, E> {
    fn lower_bounds_ref(&self) -> &Vec<Self::DataType> {
        &self.lower_bounds
    }

    fn upper_bounds_ref(&self) -> &Vec<Self::DataType> {
        &self.upper_bounds
    }

    fn lower_bounds_mut_ref(&mut self) -> &mut Vec<Self::DataType> {
        &mut self.lower_bounds
    }

    fn upper_bounds_mut_ref(&mut self) -> &mut Vec<Self::DataType> {
        &mut self.upper_bounds
    }
}

pub struct SimpleOpEnEngineConstructor;
impl<'a, D: DifferentiableFunctionTrait + 'static, E: DerivativeMethodTrait + 'static> DiffBlockObjectiveOptimizerConstructorTrait<'a, D, E> for SimpleOpEnEngineConstructor {
    type ConstructorArgsType = SimpleOpEnEngineConstructorArgs;
    type Optimizer = SimpleOpEnEngineOptimizer<'a, D, E>;

    fn construct(objective_differentiable_block: DifferentiableBlock<'a, D, E>,
                 initial_condition: Vec<f64>,
                 lower_bounds: Vec<f64>,
                 upper_bounds: Vec<f64>,
                 args: &Self::ConstructorArgsType) -> Self::Optimizer {
        let problem_size = initial_condition.len();
        SimpleOpEnEngineOptimizer {
            differentiable_block: objective_differentiable_block,
            initial_condition,
            lower_bounds,
            upper_bounds,
            cache: Mutex::new(PANOCCache::new(problem_size, args.tolerance, 3)),
        }
    }
}

#[derive(Clone, Debug)]
pub struct SimpleOpEnEngineConstructorArgs {
    pub tolerance: f64
}

/*
pub struct OptimizerCategorySimpleOpEnEngine;
impl DiffBlockOptimizerConstructorTrait for OptimizerCategorySimpleOpEnEngine {
    type Optimizer<'a, D: DifferentiableFunctionTrait + 'static, E: DerivativeMethodTrait + 'static> = SimpleOpEnEngineOptimizer<'a, D, E>;
}
*/

/*
pub struct SimpleOpEnEngineArgs<'a, D: DifferentiableFunctionTrait, E: DerivativeMethodTrait> {
    pub call_args: &'a D::ArgsType<'a, f64>,
    pub grad_args: &'a D::ArgsType<'a, E::T>,
    pub grad_method_data: &'a E::DerivativeMethodData,
    pub initial_condition: &'a [f64],
    pub lower_bounds: &'a [f64],
    pub upper_bounds: &'a [f64],
    pub cache: &'a Mutex<PANOCCache>
}
*/

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


