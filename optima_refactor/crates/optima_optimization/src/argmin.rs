use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionClass};
use argmin::core::*;

pub struct ArgminDiffBlockWrapper<'a, DC: DifferentiableFunctionClass, E: DerivativeMethodTrait> {
    diff_block: DifferentiableBlock<'a, DC, E>
}
impl<'a, DC: DifferentiableFunctionClass, E: DerivativeMethodTrait> ArgminDiffBlockWrapper<'a, DC, E> {
    pub fn diff_block(&self) -> &DifferentiableBlock<'a, DC, E> {
        &self.diff_block
    }
}
impl<'a, DC: DifferentiableFunctionClass, E: DerivativeMethodTrait> ArgminDiffBlockWrapper<'a, DC, E> {
    pub fn new(diff_block: DifferentiableBlock<'a, DC, E>) -> Self {
        Self { diff_block }
    }
}
impl<'a, DC: DifferentiableFunctionClass, E: DerivativeMethodTrait> CostFunction for ArgminDiffBlockWrapper<'a, DC, E> {
    type Param = Vec<f64>;
    type Output = f64;

    fn cost(&self, param: &Self::Param) -> Result<Self::Output, Error> {
        let res = self.diff_block.call(param);
        return Ok(res[0]);
    }
}
impl<'a, DC: DifferentiableFunctionClass, E: DerivativeMethodTrait> Gradient for ArgminDiffBlockWrapper<'a, DC, E> {
    type Param = Vec<f64>;
    type Gradient = Vec<f64>;

    fn gradient(&self, param: &Self::Param) -> Result<Self::Gradient, Error> {
        let res = self.diff_block.derivative(param);
        return Ok(res.1.as_slice().to_vec())
    }
}
/*
impl<'a, DC: DifferentiableFunctionClass, E: DerivativeMethodTrait2> Jacobian for ArgminDiffBlockWrapper<'a, DC, E> {
    type Param = Vec<f64>;
    type Jacobian = DMatrix<f64>;

    fn jacobian(&self, param: &Self::Param) -> Result<Self::Jacobian, Error> {
        let res = self.diff_block.derivative(param);
        return Ok(res.1);
    }
}
*/