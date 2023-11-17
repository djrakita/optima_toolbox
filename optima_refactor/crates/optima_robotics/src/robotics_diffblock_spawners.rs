use std::borrow::Cow;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::DerivativeMethodTrait;
use optima_3d_spatial::optima_3d_pose::O3DPoseCategoryTrait;
use optima_linalg::OLinalgCategoryTrait;
use crate::robot::ORobot;
use crate::robotics_optimization_solvers::{IKArgs, IKObjective};

pub struct RoboticsDiffBlockSpawners;
impl RoboticsDiffBlockSpawners {
    pub fn spawn_ik_diff_block<E: DerivativeMethodTrait, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static>( robots: &(ORobot<f64, C, L>, ORobot<E::T, C, L>), derivative_method_data: E::DerivativeMethodData ) -> DifferentiableBlock<IKObjective<C, L>, E> {
        let args1 = IKArgs {
            robot: Cow::Borrowed(&robots.0),
            goals: vec![],
        };

        let args2 = IKArgs {
            robot: Cow::Borrowed(&robots.1),
            goals: vec![],
        };

        DifferentiableBlock::new(args1, args2, derivative_method_data)
    }
}