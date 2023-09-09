/*
use std::marker::PhantomData;
use ad_trait::AD;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait};
use optima_3d_spatial::optima_3d_pose::O3DPoseCategoryTrait;
use optima_linalg::OLinalgTrait;
use optima_optimization::derivative_based_optimization::{DerivBasedOptSolver};
use crate::robot::ORobot;

pub struct IKFunction<C: O3DPoseCategoryTrait, L: OLinalgTrait>(PhantomData<(C, L)>);
impl<C: O3DPoseCategoryTrait, L: OLinalgTrait> DifferentiableFunctionTrait for IKFunction<C, L> {
    type ArgsType<T: AD> = ORobot<T, C::P<T>, L>;

    fn call<T1: AD>(inputs: &[T1], args: &Self::ArgsType<T1>) -> Vec<T1> {
        todo!()
    }

    fn num_inputs<T1: AD>(args: &Self::ArgsType<T1>) -> usize {
        todo!()
    }

    fn num_outputs<T1: AD>(args: &Self::ArgsType<T1>) -> usize {
        todo!()
    }
}

pub type DerivBasedIKSolver<C, L, E, O> = DerivBasedOptSolver<IKFunction<C, L>, E, O>;
*/