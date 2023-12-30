use std::time::{Duration, Instant};
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionClass};
pub use nlopt::{Algorithm, Nlopt, SuccessState, Target};
use crate::{DiffBlockOptimizerTrait, OptimizerOutputTrait};

pub struct NLOptOptimizer {
    algorithm: Algorithm,
    use_constraints: bool,
    problem_size: usize,
    max_iter: Option<u32>,
    lower_bounds: Option<Vec<f64>>,
    upper_bounds: Option<Vec<f64>>
}
impl NLOptOptimizer {
    pub fn new(algorithm: Algorithm, use_constraints: bool, problem_size: usize, max_iter: Option<u32>, lower_bounds: Option<Vec<f64>>, upper_bounds: Option<Vec<f64>>) -> Self {
        Self { algorithm, use_constraints, problem_size, max_iter, lower_bounds, upper_bounds }
    }
}
impl DiffBlockOptimizerTrait for NLOptOptimizer {
    type OutputType = NLOptOptimizerOutput;

    fn optimize<'a, DC1, E1, DC2, E2, DC3, E3>(&self, initial_condition: &[f64], objective_function: &DifferentiableBlock<'a, DC1, E1>, equality_constraint_function: &DifferentiableBlock<'a, DC2, E2>, inequality_constraint_function: &DifferentiableBlock<'a, DC3, E3>) -> Self::OutputType where DC1: DifferentiableFunctionClass, DC2: DifferentiableFunctionClass, DC3: DifferentiableFunctionClass, E1: DerivativeMethodTrait, E2: DerivativeMethodTrait, E3: DerivativeMethodTrait {
        let start = Instant::now();
        let obj_f = |x: &[f64], gradient: Option<&mut [f64]>, _params: &mut ()| -> f64 {
            let res = objective_function.call(x);

            if let Some(gradient) = gradient {
                let res = objective_function.derivative(x);
                let g = res.1.data.as_vec();
                assert_eq!(g.len(), gradient.len());
                gradient.iter_mut().zip(g.iter()).for_each(|(x, y)| *x=*y);
            }

            res[0]
        };

        let mut nlopt = Nlopt::new(self.algorithm.clone(), self.problem_size, obj_f, Target::Minimize, ());

        if self.use_constraints {
            let eq_c = |x: &[f64], gradient: Option<&mut [f64]>, _params: &mut ()| -> f64 {
                let res = equality_constraint_function.call(x);

                if let Some(gradient) = gradient {
                    let res = equality_constraint_function.derivative(x);
                    let g = res.1.data.as_vec();
                    assert_eq!(g.len(), gradient.len());
                    gradient.iter_mut().zip(g.iter()).for_each(|(x, y)| *x = *y);
                }

                res[0]
            };
            let ineq_c = |x: &[f64], gradient: Option<&mut [f64]>, _params: &mut ()| -> f64 {
                let res = inequality_constraint_function.call(x);

                if let Some(gradient) = gradient {
                    let res = inequality_constraint_function.derivative(x);
                    let g = res.1.data.as_vec();
                    assert_eq!(g.len(), gradient.len());
                    gradient.iter_mut().zip(g.iter()).for_each(|(x, y)| *x = *y);
                }

                res[0]
            };

            nlopt.add_equality_constraint(eq_c, (), 0.0001).expect("error");
            nlopt.add_inequality_constraint(ineq_c, (), 0.0001).expect("error");
        }

        if let Some(lower_bounds) = &self.lower_bounds {
            nlopt.set_lower_bounds(lower_bounds).expect("error");
        }
        if let Some(upper_bounds) = &self.upper_bounds {
            nlopt.set_upper_bounds(upper_bounds).expect("error");
        }
        if let Some(max_iter) = &self.max_iter {
            nlopt.set_maxeval(*max_iter).expect("error");
        }

        nlopt.set_ftol_rel(0.001).expect("error");

        let mut x = initial_condition.to_vec();
        let res = nlopt.optimize(&mut x);
        match res {
            Ok(res) => {
                return NLOptOptimizerOutput {
                    x_star: x.clone(),
                    f_star: res.1,
                    success_state: res.0,
                    solve_time: start.elapsed(),
                };
            }
            Err(e) => {
                panic!("Optimization failed: {:?}", e)
            }
        }
    }
}

#[derive(Clone, Debug)]
pub struct NLOptOptimizerOutput {
    x_star: Vec<f64>,
    f_star: f64,
    success_state: SuccessState,
    solve_time: Duration
}
impl NLOptOptimizerOutput {
    #[inline(always)]
    pub fn success_state(&self) -> SuccessState {
        self.success_state
    }
    #[inline(always)]
    pub fn solve_time(&self) -> Duration {
        self.solve_time
    }
}
impl OptimizerOutputTrait for NLOptOptimizerOutput {
    type DataType = f64;

    #[inline(always)]
    fn x_star(&self) -> &[Self::DataType] {
        &self.x_star
    }

    #[inline(always)]
    fn f_star(&self) -> Self::DataType {
        self.f_star
    }
}