use std::sync::Mutex;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};
use optimization_engine::core::SolverStatus;
pub use optimization_engine::panoc::{PANOCCache, PANOCOptimizer};
pub use optimization_engine::{constraints, Optimizer, Problem, SolverError};
use crate::{ADTraitBasedOptimizerTrait, OptimizerOutputTrait, OptimizerTrait};

pub struct SimpleOpEnEngine<'a, D: DifferentiableFunctionTrait + 'static, E: DerivativeMethodTrait + 'static> {
    pub call_args: D::ArgsType<'a, f64>,
    pub derivative_args: D::ArgsType<'a, E::T>,
    pub derivative_method_data: E::DerivativeMethodData,
    pub initial_condition: Vec<f64>,
    pub lower_bounds: Vec<f64>,
    pub upper_bounds: Vec<f64>,
    pub cache: Mutex<PANOCCache>
}
impl<'a, D: DifferentiableFunctionTrait, E: DerivativeMethodTrait> OptimizerTrait for SimpleOpEnEngine<'a, D, E> {
    type DataType = f64;
    type OutputType = SimpleOpEnEngineOutput;

    fn optimize(&self) -> Self::OutputType {
        let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
            let res = D::derivative::<E>(u, &self.derivative_args, &self.derivative_method_data);
            let grad_as_slice = res.1.as_slice();
            assert_eq!(grad_as_slice.len(), grad.len());
            grad.iter_mut().zip(grad_as_slice.iter()).for_each(|(x, y)| *x = *y );
            Ok(())
        };
        let f = |u: &[f64], cost: &mut f64| -> Result<(), SolverError> {
            let res = D::call(u, &self.call_args);
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
        
        SimpleOpEnEngineOutput {
            x_star: x,
            solver_status,
        }
    }
}
impl<'a, D: DifferentiableFunctionTrait, E: DerivativeMethodTrait> ADTraitBasedOptimizerTrait<D, E> for SimpleOpEnEngine<'a, D, E> {
    fn update_objective_args<U: Fn(&mut D::ArgsType<'_, f64>, &mut D::ArgsType<'_, E::T>)>(&mut self, update_fn: U) {
        (update_fn)(&mut self.call_args, &mut self.derivative_args)
    }

    fn update_initial_condition(&mut self, v: &[Self::DataType]) {
        assert_eq!(v.len(), self.initial_condition.len());
        self.initial_condition = v.to_vec();
    }
}

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

pub struct SimpleOpEnEngineOutput {
    x_star: Vec<f64>,
    solver_status: SolverStatus
}
impl SimpleOpEnEngineOutput {
    #[inline(always)]
    pub fn solver_status(&self) -> SolverStatus {
        self.solver_status
    }
}
impl OptimizerOutputTrait for SimpleOpEnEngineOutput {
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