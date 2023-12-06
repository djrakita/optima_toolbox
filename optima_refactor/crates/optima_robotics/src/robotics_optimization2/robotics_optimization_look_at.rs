use std::marker::PhantomData;
use std::sync::RwLock;
use ad_trait::AD;
use ad_trait::differentiable_block::DifferentiableBlock2;
use ad_trait::differentiable_function::{DerivativeMethodTrait2, DifferentiableFunctionClass, DifferentiableFunctionTrait2};
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategory};
use optima_3d_spatial::optima_3d_vec::O3DVecCategoryArr;
use optima_linalg::{OLinalgCategory, OVec, OVecCategoryVec};
use optima_optimization::loss_functions::{GrooveLossGaussianDirection, OptimizationLossFunctionTrait, OptimizationLossGroove};
use optima_proximity::pair_group_queries::{OPairGroupQryTrait, PairGroupQryOutputCategoryParryDistance, PairGroupQryOutputCategoryParryFilter, ParryPairSelector};
use optima_proximity::shapes::ShapeCategoryOParryShape;
use crate::robotics_optimization2::robotics_optimization_functions::{LookAtAxis, LookAtTarget, robot_link_look_at_objective};
use crate::robotics_optimization2::robotics_optimization_ik::{DifferentiableFunctionIKObjective, IKGoalRwLockVecTrait, IKGoalUpdateMode, IKPrevStatesRwLockTrait};

pub struct DifferentiableFunctionClassLookAt<C, L, FQ, Q>(PhantomData<(C, L, FQ, Q)>)
    where
        C: O3DPoseCategory + 'static,
        L: OLinalgCategory + 'static,
        FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
        Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance>;
impl<C, L, FQ, Q> DifferentiableFunctionClass for DifferentiableFunctionClassLookAt<C, L, FQ, Q>
    where
        C: O3DPoseCategory + 'static,
        L: OLinalgCategory + 'static,
        FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
        Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance>
{
    type FunctionType<'a, T: AD> = DifferentiableFunctionLookAt<'a, T, C, L, FQ, Q>;
}

pub struct DifferentiableFunctionLookAt<'a, T, C, L, FQ, Q>
    where T: AD,
          C: O3DPoseCategory + 'static,
          L: OLinalgCategory + 'static,
          FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
          Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance> {
    ik_objective: DifferentiableFunctionIKObjective<'a, T, C, L, FQ, Q>,
    looker_link: usize,
    looker_axis: LookAtAxis,
    look_at_target: RwLock<LookAtTarget<T, O3DVecCategoryArr>>,
    look_at_weight: T
}
impl<'a, T, C, L, FQ, Q> DifferentiableFunctionLookAt<'a, T, C, L, FQ, Q> where T: AD,
                                                                                C: O3DPoseCategory + 'static,
                                                                                L: OLinalgCategory + 'static,
                                                                                FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
                                                                                Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance> {
    pub fn new(ik_objective: DifferentiableFunctionIKObjective<'a, T, C, L, FQ, Q>, looker_link: usize, looker_axis: LookAtAxis, look_at_target: LookAtTarget<T, O3DVecCategoryArr>, look_at_weight: T) -> Self {
        Self { ik_objective, looker_link, looker_axis, look_at_target: RwLock::new(look_at_target), look_at_weight }
    }
}
impl<'a, T, C, L, FQ, Q> DifferentiableFunctionTrait2<'a, T> for DifferentiableFunctionLookAt<'a, T, C, L, FQ, Q>
    where T: AD,
          C: O3DPoseCategory + 'static,
          L: OLinalgCategory + 'static,
          FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
          Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance>
{
    fn call(&self, inputs: &[T]) -> Vec<T> {
        let (output, fk_res) = self.ik_objective.call_and_return_fk_res(inputs);

        let mut out = output[0];
        let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T::zero(), T::constant(2.0), T::constant(0.2), T::constant(1.0), T::constant(2.0));
        out += self.look_at_weight*loss.loss(robot_link_look_at_objective::<T, C>(&fk_res, self.looker_link, &self.looker_axis, &self.look_at_target.read().unwrap()));

        vec![out]
    }

    fn num_inputs(&self) -> usize {
        self.ik_objective.robot().num_dofs()
    }

    fn num_outputs(&self) -> usize {
        1
    }
}

pub type DifferentiableBlockLookAt<'a, E, C, L, FQ, Q> = DifferentiableBlock2<'a, DifferentiableFunctionClassLookAt<C, L, FQ, Q>, E>;
pub trait DifferentiableBlockLookAtTrait<'a, C: O3DPoseCategory> {
    fn update_look_at_target(&self, look_at_target: LookAtTarget<f64, O3DVecCategoryArr>);
    fn update_ik_pose(&self, idx: usize, pose: C::P<f64>, update_mode: IKGoalUpdateMode);
    fn update_prev_states(&self, state: Vec<f64>);
}
impl<'a, E, C, L, FQ, Q> DifferentiableBlockLookAtTrait<'a, C> for DifferentiableBlockLookAt<'a, E, C, L, FQ, Q>
    where E: DerivativeMethodTrait2,
          C: O3DPoseCategory + 'static,
          L: OLinalgCategory + 'static,
          FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
          Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance>
{
    fn update_look_at_target(&self, look_at_target: LookAtTarget<f64, O3DVecCategoryArr>) {
        self.update_function(|x, y| {
            *y.look_at_target.write().unwrap() = look_at_target.to_new_ad_type::<E::T>();
            *x.look_at_target.write().unwrap() =  look_at_target.to_owned();
        });
    }

    fn update_ik_pose(&self, idx: usize, pose: C::P<f64>, update_mode: IKGoalUpdateMode) {
        self.update_function(|x, y| {
            x.ik_objective.ik_goals().update_ik_goal(idx, pose.clone(), update_mode.clone());
            y.ik_objective.ik_goals().update_ik_goal(idx, pose.o3dpose_to_other_generic_category::<E::T, C>(), update_mode.clone());
        });
    }

    fn update_prev_states(&self, state: Vec<f64>) {
        self.update_function(|x, y| {
            x.ik_objective.prev_states().update(state.clone());
            y.ik_objective.prev_states().update(state.ovec_to_other_generic_category::<E::T, OVecCategoryVec>());
        });
    }
}
