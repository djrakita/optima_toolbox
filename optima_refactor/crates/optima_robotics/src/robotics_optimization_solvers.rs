use std::marker::PhantomData;
use ad_trait::AD;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait};
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryTrait};
use optima_linalg::OLinalgTrait;
use optima_optimization::derivative_based_optimization::{DerivBasedOptSolver};
use crate::robot::ORobot;

pub struct SimpleIKArgs<T: AD, P: O3DPose<T>, L: OLinalgTrait> {
    robot: ORobot<T, P, L>,
    pub chain_idx: usize,
    pub link_idx: usize,
    pub goal_pose: P
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> SimpleIKArgs<T, P, L> {
    pub fn new(robot: ORobot<T, P, L>, chain_idx: usize, link_idx: usize, goal_pose: P) -> Self {
        Self {
            robot,
            chain_idx,
            link_idx,
            goal_pose,
        }
    }
    pub fn robot(&self) -> &ORobot<T, P, L> {
        &self.robot
    }
}

pub struct SimpleIKFunction<C: O3DPoseCategoryTrait, L: OLinalgTrait>(PhantomData<(C, L)>);
impl<C: O3DPoseCategoryTrait, L: OLinalgTrait> DifferentiableFunctionTrait for SimpleIKFunction<C, L> {
    type ArgsType<T: AD> = SimpleIKArgs<T, C::P<T>, L>;

    fn call<T1: AD>(inputs: &[T1], args: &Self::ArgsType<T1>) -> Vec<T1> {
        let fk_res = args.robot.forward_kinematics(&inputs, None);
        let pose = fk_res.get_chain_fk_result(args.chain_idx).as_ref().expect("error").get_link_pose(args.link_idx).as_ref().expect("error");
        let dis = pose.dis(&args.goal_pose);

        return vec![dis.powi(2)]
    }

    fn num_inputs<T1: AD>(args: &Self::ArgsType<T1>) -> usize {
        args.robot.num_dofs()
    }

    fn num_outputs<T1: AD>(_args: &Self::ArgsType<T1>) -> usize {
        1
    }
}

pub type SimpleIKSolver<C, L, E, O> = DerivBasedOptSolver<SimpleIKFunction<C, L>, E, O>;