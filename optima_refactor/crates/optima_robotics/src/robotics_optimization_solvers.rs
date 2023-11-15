use std::marker::PhantomData;
use ad_trait::AD;
use ad_trait::differentiable_function::DifferentiableFunctionTrait;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryTrait};
use optima_linalg::OLinalgCategoryTrait;
use crate::robot::ORobot;

pub struct IKObjective<C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static>(PhantomData<(C, L)>);
impl<C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static> DifferentiableFunctionTrait for IKObjective<C, L> {
    type ArgsType<'a, T: AD> = IKArgs<'a, T, C, L>;

    fn call<'a, T1: AD>(inputs: &[T1], args: &Self::ArgsType<'a, T1>) -> Vec<T1> {
        let fk_res = args.robot.forward_kinematics(&inputs.to_vec(), None);

        let mut out = T1::zero();
        args.goals.iter().for_each(|g| {
            let pose = fk_res.get_link_pose(g.goal_link_idx).as_ref().expect("error");
            let dis = pose.dis(&g.goal_pose);
            out += g.weight * dis;
        });

        vec![out]
    }

    fn num_inputs<T1: AD>(args: &Self::ArgsType<'_, T1>) -> usize {
        args.robot.num_dofs()
    }

    fn num_outputs<T1: AD>(_args: &Self::ArgsType<'_, T1>) -> usize {
        1
    }
}

pub struct IKArgs<'a, T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static> {
    pub robot: &'a ORobot<T, C, L>,
    pub goals: Vec<IKGoal<T, C::P<T>>>
}

pub struct IKGoal<T: AD, P: O3DPose<T>> {
    pub goal_link_idx: usize,
    pub goal_pose: P,
    pub weight: T,
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
use std::marker::PhantomData;
use ad_trait::AD;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait};
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryTrait};
use optima_linalg::OLinalgCategoryTrait;
use optima_optimization::derivative_based_optimization::{DerivBasedOptSolver};
use crate::robot::ORobot;

pub struct SimpleIKArgs<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> {
    robot: ORobot<T, C, L>,
    pub chain_idx: usize,
    pub link_idx: usize,
    pub goal_pose: C::P<T>
}
impl<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> SimpleIKArgs<T, C, L> {
    pub fn new(robot: ORobot<T, C, L>, chain_idx: usize, link_idx: usize, goal_pose: C::P<T>) -> Self {
        Self {
            robot,
            chain_idx,
            link_idx,
            goal_pose,
        }
    }
    pub fn robot(&self) -> &ORobot<T, C, L> {
        &self.robot
    }
}

pub struct SimpleIKFunction<C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait>(PhantomData<(C, L)>);
impl<C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> DifferentiableFunctionTrait for SimpleIKFunction<C, L> {
    type ArgsType<T: AD> = SimpleIKArgs<T, C, L>;

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
*/