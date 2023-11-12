pub mod optimization_engine;

use std::any::Any;
use ad_trait::AD;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};

pub trait OptimizerTrait {
    type DataType : AD;
    type OutputType : Any + OptimizerOutputTrait<DataType = Self::DataType>;

    fn optimize(&self) -> Self::OutputType;
}

pub trait OptimizerDiffBlockObjectiveFunctionTrait<'a>: OptimizerTrait {
    type D : DifferentiableFunctionTrait;
    type E : DerivativeMethodTrait;

    fn objective_function_differentiable_block(&self) -> &DifferentiableBlock<'a, Self::D, Self::E>;
    fn objective_function_differentiable_block_mut(&mut self) -> &mut DifferentiableBlock<'a, Self::D, Self::E>;

    /*
    #[inline]
    fn update_objective_function_args<U>(&'a mut self, u: U) where U: Fn(&'a mut <Self::D as DifferentiableFunctionTrait>::ArgsType<'a, f64>, &'a mut <Self::D as DifferentiableFunctionTrait>::ArgsType<'a, <Self::E as DerivativeMethodTrait>::T>) {
        let a = self.objective_args_mut();
        (u)(a.0, a.1)
    }
    */
}

pub trait OptimizerBoundsTrait : OptimizerTrait {
    fn lower_bounds_ref(&self) -> &Vec<Self::DataType>;
    fn upper_bounds_ref(&self) -> &Vec<Self::DataType>;
    fn lower_bounds_mut_ref(&mut self) -> &mut Vec<Self::DataType>;
    fn upper_bounds_mut_ref(&mut self) -> &mut Vec<Self::DataType>;
}

pub trait OptimizerInitialConditionTrait : OptimizerTrait {
    fn initial_condition_ref(&self) -> &Vec<Self::DataType>;
    fn initial_condition_mut(&mut self) -> &mut Vec<Self::DataType>;

    #[inline(always)]
    fn update_initial_condition(&mut self, initial_condition: &[Self::DataType]) {
        let self_initial_condition = self.initial_condition_mut();
        assert_eq!(initial_condition.len(), self_initial_condition.len());
        *self_initial_condition = initial_condition.to_vec();
    }
}

pub trait OptimizerOutputTrait {
    type DataType : AD;

    fn x_star(&self) -> &[Self::DataType];
    fn f_star(&self) -> Self::DataType;
}
impl<O: OptimizerOutputTrait> OptimizerOutputTrait for Box<O> {
    type DataType = O::DataType;

    #[inline(always)]
    fn x_star(&self) -> &[Self::DataType] {
        self.as_ref().x_star()
    }

    #[inline(always)]
    fn f_star(&self) -> Self::DataType {
        self.as_ref().f_star()
    }
}

pub trait DiffBlockObjectiveOptimizerConstructorTrait<'a, D: DifferentiableFunctionTrait + 'static, E: DerivativeMethodTrait + 'static> {
    type ConstructorArgsType;
    type Optimizer : OptimizerDiffBlockObjectiveFunctionTrait<'a>;

    fn construct(objective_differentiable_block: DifferentiableBlock<'a, D, E>,
                 initial_condition: Vec<f64>,
                 lower_bounds: Vec<f64>,
                 upper_bounds: Vec<f64>,
                 args: &Self::ConstructorArgsType) -> Self::Optimizer;
}

/*
pub struct OptimizationSolver<'a, O: OptimizerTrait> {
    optimizer: O,
    phantom_data: PhantomData<&'a ()>
}
impl<'a, O: OptimizerTrait> OptimizationSolver<'a, O> {
    pub fn optimize(&self) -> O::OutputType {
        self.optimizer.optimize()
    }
}
impl<'a, O: OptimizerDiffBlockObjectiveFunctionTrait<'a>> OptimizationSolver<'a, O> {
    #[inline]
    pub fn update_objective_function_args<U>(&'a mut self, u: U) where U: Fn(&'a mut <O::D as DifferentiableFunctionTrait>::ArgsType<'a, f64>, &'a mut <O::D as DifferentiableFunctionTrait>::ArgsType<'a, <O::E as DerivativeMethodTrait>::T>) {
        let a = self.optimizer.objective_args_mut();
        (u)(a.0, a.1)
    }
}
impl<'a, O: OptimizerInitialConditionTrait> OptimizationSolver<'a, O> {
    #[inline(always)]
    pub fn update_initial_condition(&mut self, initial_condition: &'a [O::DataType]) {
        let self_initial_condition = self.optimizer.initial_condition_mut();
        assert_eq!(initial_condition.len(), self_initial_condition.len());
        *self_initial_condition = initial_condition.to_vec();
    }
}
*/
/*
impl<'a, D, E, O> OptimizationSolver<'a, D, E, O>
    where O: OptimizerTrait
{

}
*/
/*
impl<'a, D, E, O> OptimizationSolver<'a, D, E, O>
    where
        O: ADTraitBasedObjectiveFunctionTrait<'a, D, E>,
        D: DifferentiableFunctionTrait + 'static,
        E: DerivativeMethodTrait + 'static,
{

}
*/






