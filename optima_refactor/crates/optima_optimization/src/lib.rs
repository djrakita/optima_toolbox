pub mod optimization_engine;

use std::any::Any;
use ad_trait::AD;
use ad_trait::differentiable_block::{DifferentiableBlock, DifferentiableBlockArgPrepTrait};
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};

pub trait DiffBlockUnconstrainedOptimizerTrait {
    type DataType : AD;
    type OutputType : Any + OptimizerOutputTrait<DataType = Self::DataType>;

    fn diff_block_unconstrained_optimize<'a, D1: DifferentiableFunctionTrait, E1: DerivativeMethodTrait, AP: DifferentiableBlockArgPrepTrait<'a, D1, E1>>(&self, objective_function: &DifferentiableBlock<'a, D1, E1, AP>, initial_condition: &[Self::DataType]) -> Self::OutputType;
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

pub trait OptimizationLossFunctionTrait<T: AD> {
    fn loss(&self, val: T) -> T;
}

pub struct OptimizationLossQuadratic<T: AD> {
    a: T
}
impl<T: AD> OptimizationLossQuadratic<T> {
    pub fn new(a: T) -> Self {
        Self { a }
    }
}
impl<T: AD> OptimizationLossFunctionTrait<T> for OptimizationLossQuadratic<T> {
    fn loss(&self, val: T) -> T {
        return (self.a * val).powi(2)
    }
}

pub struct OptimizationLossGroove<T: AD> {
    gaussian_direction: T,
    center: T,
    gaussian_exponent: T,
    gaussian_spread: T,
    polynomial_weight: T,
    polynomial_exponent: T,
}
impl<T: AD> OptimizationLossGroove<T> {
    pub fn new(gaussian_direction: GrooveLossGaussianDirection, center: T, gaussian_exponent: T, gaussian_spread: T, polynomial_weight: T, polynomial_exponent: T) -> Self {
        let gaussian_direction = match gaussian_direction {
            GrooveLossGaussianDirection::BowlUp => { T::one() }
            GrooveLossGaussianDirection::BowlDown => { T::zero() }
        };
        Self { gaussian_direction, center, gaussian_exponent, gaussian_spread, polynomial_weight, polynomial_exponent }
    }
}
impl<T: AD> OptimizationLossFunctionTrait<T> for OptimizationLossGroove<T> {
    fn loss(&self, val: T) -> T {
        let two = T::constant(2.0);
        let val1 = -T::one().powf(self.gaussian_direction);
        let val2 = -(val - self.center).powf(self.gaussian_exponent);
        let val3 = two*self.gaussian_spread.powf(two);
        let val4 = val1*(val2 / val3).exp();
        let val5 = self.polynomial_weight *(val - self.center).powf(self.polynomial_exponent);
        return val4 + val5;
    }
}

#[derive(Clone, Debug)]
pub enum GrooveLossGaussianDirection {
    BowlUp, BowlDown
}