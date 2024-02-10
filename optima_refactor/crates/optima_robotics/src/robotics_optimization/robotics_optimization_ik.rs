use std::marker::PhantomData;
use std::sync::{Arc, RwLock};
use ad_trait::AD;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionClass, DifferentiableFunctionTrait};
use serde::{Deserialize, Serialize};
use serde_with::serde_as;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategory, AliasO3DPoseCategory};
use optima_linalg::{OVec, OVecCategoryVec, AliasOLinalgCategory};
use optima_proximity::pair_group_queries::{OwnedPairGroupQry, OParryFilterOutput, OParryPairSelector, OProximityLossFunction};
use crate::robot::{FKResult, ORobot};
use crate::robotics_optimization::robotics_optimization_functions::{robot_ik_position_goals_objective, robot_ik_rotation_goals_objective, robot_per_instant_velocity_acceleration_and_jerk_objectives, robot_self_proximity_objective, robot_self_proximity_refilter_check};
use optima_3d_spatial::optima_3d_pose::SerdeO3DPose;
use ad_trait::SerdeAD;
use serde_with::*;
use optima_file::traits::{FromJsonString, ToJsonString};
use optima_optimization::loss_functions::{GrooveLossGaussianDirection, OptimizationLossFunctionTrait, OptimizationLossGroove};
use optima_proximity::trait_aliases::{AliasParryGroupFilterQry, AliasParryToProximityQry};

pub struct DifferentiableFunctionClassIKObjective<C, L, FQ, Q>(PhantomData<(C, L, FQ, Q)>)
    where C: AliasO3DPoseCategory,
          L: AliasOLinalgCategory,
          FQ: AliasParryGroupFilterQry,
          Q: AliasParryToProximityQry;
impl<C, L, FQ, Q> DifferentiableFunctionClassIKObjective<C, L, FQ, Q> where C: AliasO3DPoseCategory,
                                                                            L: AliasOLinalgCategory,
                                                                            FQ: AliasParryGroupFilterQry,
                                                                            Q: AliasParryToProximityQry {
    pub fn new(_pose_category: C, _linalg_category: L, _filter_query: FQ, _distance_query: Q) -> Self {
        Self(PhantomData::default())
    }
}
impl<C, L, FQ, Q> DifferentiableFunctionClass for DifferentiableFunctionClassIKObjective<C, L, FQ, Q>
    where C: AliasO3DPoseCategory,
          L: AliasOLinalgCategory,
          FQ: AliasParryGroupFilterQry,
          Q: AliasParryToProximityQry {
    type FunctionType<T: AD> = DifferentiableFunctionIKObjective<T, C, L, FQ, Q>;
}

pub struct DifferentiableFunctionIKObjective<T, C, L, FQ, Q>
    where T:AD,
          C: AliasO3DPoseCategory,
          L: AliasOLinalgCategory,
          FQ: AliasParryGroupFilterQry,
          Q: AliasParryToProximityQry {
    robot: Arc<ORobot<T, C, L>>,
    ik_goals: RwLock<Vec<IKGoal<T, C::P<T>>>>,
    prev_states: RwLock<IKPrevStates<T>>,
    filter_query: OwnedPairGroupQry<T, FQ>,
    distance_query: OwnedPairGroupQry<T, Q>,
    constant_selector: Option<OParryPairSelector>,
    dis_filter_cutoff: T,
    linf_dis_cutoff: f64,
    last_proximity_filter_state: Arc<RwLock<Option<Vec<f64>>>>,
    filter_output: Arc<RwLock<Option<OParryFilterOutput>>>,
    ee_matching_weight: T,
    collision_avoidance_weight: T,
    min_vel_weight: T,
    min_acc_weight: T,
    min_jerk_weight: T
}
impl<T, C, L, FQ, Q> DifferentiableFunctionIKObjective<T, C, L, FQ, Q> where T: AD,
                                                                                     C: AliasO3DPoseCategory,
                                                                                     L: AliasOLinalgCategory,
                                                                                     FQ: AliasParryGroupFilterQry,
                                                                                     Q: AliasParryToProximityQry {
    pub fn new(robot: Arc<ORobot<T, C, L>>, ik_goals: Vec<IKGoal<T, C::P<T>>>, init_state: Vec<T>, filter_query: OwnedPairGroupQry<T, FQ>, distance_query: OwnedPairGroupQry<T, Q>, constant_selector: Option<OParryPairSelector>, dis_filter_cutoff: T, linf_dis_cutoff: f64, last_proximity_filter_state: Arc<RwLock<Option<Vec<f64>>>>, filter_output: Arc<RwLock<Option<OParryFilterOutput>>>, ee_matching_weight: T, collision_avoidance_weight: T, min_vel_weight: T, min_acc_weight: T, min_jerk_weight: T) -> Self {
        let prev_states = IKPrevStates::new(init_state.clone());
        Self { robot, ik_goals: RwLock::new(ik_goals), prev_states: RwLock::new(prev_states), filter_query, distance_query, constant_selector, dis_filter_cutoff, linf_dis_cutoff, last_proximity_filter_state, filter_output, ee_matching_weight, collision_avoidance_weight, min_vel_weight, min_acc_weight, min_jerk_weight }
    }
    pub fn call_and_return_fk_res(&self, inputs: &[T], freeze: bool) -> (Vec<T>, FKResult<T, C::P<T>>) {
        let inputs_as_vec = inputs.to_vec();
        let fk_res = self.robot.forward_kinematics(&inputs_as_vec, None);

        if self.collision_avoidance_weight > T::zero() && self.constant_selector.is_none() {
            robot_self_proximity_refilter_check(&self.robot, &self.filter_query, inputs, &fk_res, &self.last_proximity_filter_state, &self.filter_output, self.linf_dis_cutoff);
        }

        let mut out_val = T::zero();

        if self.ee_matching_weight > T::zero() {
            let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T::zero(), T::constant(2.0), T::constant(0.5), T::constant(1.0), T::constant(2.0));
            out_val += self.ee_matching_weight * loss.loss(robot_ik_position_goals_objective::<T, C>(&fk_res, &self.ik_goals.read().unwrap()));
            out_val += self.ee_matching_weight * loss.loss(robot_ik_rotation_goals_objective::<T, C>(&fk_res, &self.ik_goals.read().unwrap()));
        }

        if self.collision_avoidance_weight > T::zero() {
            let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T::zero(), T::constant(6.0), T::constant(0.4), T::constant(2.0), T::constant(4.0));
            let tmp = match &self.constant_selector {
                None => {
                    let binding = self.filter_output.read().unwrap();
                    let selector = binding.as_ref().unwrap().selector();
                    robot_self_proximity_objective(&self.robot, &fk_res, &self.distance_query, selector, self.dis_filter_cutoff, T::constant(15.0), OProximityLossFunction::Hinge, freeze)
                }
                Some(selector) => {
                    robot_self_proximity_objective(&self.robot, &fk_res, &self.distance_query, selector, self.dis_filter_cutoff, T::constant(15.0), OProximityLossFunction::Hinge, freeze)
                }
            }.powi(2);
            // println!("{:?}", tmp);
            let tmp = self.collision_avoidance_weight * loss.loss(tmp);
            // println!("...{:?}", tmp);
            out_val += tmp;
        }

        if self.min_vel_weight + self.min_acc_weight + self.min_jerk_weight > T::zero() {
            let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T::zero(), T::constant(2.0), T::constant(0.5), T::constant(2.0), T::constant(2.0));
            let (v, a, j) = robot_per_instant_velocity_acceleration_and_jerk_objectives(inputs, &self.prev_states.read().unwrap(), T::constant(10.0));

            out_val += self.min_vel_weight * loss.loss(v);
            out_val += self.min_acc_weight * loss.loss(a);
            out_val += self.min_jerk_weight * loss.loss(j);
        }

        (vec![out_val], fk_res)
    }
    pub fn robot(&self) -> &Arc<ORobot<T, C, L>> {
        &self.robot
    }
    pub fn ik_goals(&self) -> &RwLock<Vec<IKGoal<T, C::P<T>>>> {
        &self.ik_goals
    }
    pub fn prev_states(&self) -> &RwLock<IKPrevStates<T>> {
        &self.prev_states
    }
    pub fn to_other_ad_type<T1: AD>(&self) -> DifferentiableFunctionIKObjective<T1, C, L, FQ, Q> {
        DifferentiableFunctionIKObjective {
            robot: Arc::new(self.robot.to_other_ad_type::<T1>()),
            ik_goals: self.ik_goals.to_other_generic_types::<T1, C>(),
            prev_states: self.prev_states.to_other_ad_type::<T1>(),
            filter_query: self.filter_query.to_other_ad_type::<T1>(),
            distance_query: self.distance_query.to_other_ad_type::<T1>(),
            constant_selector: self.constant_selector.clone(),
            dis_filter_cutoff: self.dis_filter_cutoff.to_other_ad_type::<T1>(),
            linf_dis_cutoff: self.linf_dis_cutoff.clone(),
            last_proximity_filter_state: self.last_proximity_filter_state.clone(),
            filter_output: self.filter_output.clone(),
            ee_matching_weight: self.ee_matching_weight.to_other_ad_type::<T1>(),
            collision_avoidance_weight: self.collision_avoidance_weight.to_other_ad_type::<T1>(),
            min_vel_weight: self.min_vel_weight.to_other_ad_type::<T1>(),
            min_acc_weight: self.min_acc_weight.to_other_ad_type::<T1>(),
            min_jerk_weight: self.min_jerk_weight.to_other_ad_type::<T1>(),
        }
    }
}
impl<T, C, L, FQ, Q> DifferentiableFunctionTrait<T> for DifferentiableFunctionIKObjective<T, C, L, FQ, Q> where T: AD,
                                                                                                                            C: AliasO3DPoseCategory,
                                                                                                                            L: AliasOLinalgCategory,
                                                                                                                            FQ: AliasParryGroupFilterQry,
                                                                                                                            Q: AliasParryToProximityQry {
    fn call(&self, inputs: &[T], freeze: bool) -> Vec<T> {
        self.call_and_return_fk_res(inputs, freeze).0
    }

    fn num_inputs(&self) -> usize {
        self.robot.num_dofs()
    }

    fn num_outputs(&self) -> usize { 1 }
}

pub type DifferentiableBlockIKObjective<C, L, FQ, Q, E> = DifferentiableBlock<DifferentiableFunctionClassIKObjective<C, L, FQ, Q>, E>;
pub trait DifferentiableBlockIKObjectiveTrait<C: O3DPoseCategory> {
    fn update_ik_pose(&self, idx: usize, pose: C::P<f64>, update_mode: IKGoalUpdateMode);
    fn update_prev_states(&self, state: Vec<f64>);
}
impl<C, L, FQ, Q, E> DifferentiableBlockIKObjectiveTrait<C> for DifferentiableBlock<DifferentiableFunctionClassIKObjective<C, L, FQ, Q>, E>
    where C: AliasO3DPoseCategory,
          L: AliasOLinalgCategory,
          FQ: AliasParryGroupFilterQry,
          Q: AliasParryToProximityQry,
          E: DerivativeMethodTrait
{
    #[inline]
    fn update_ik_pose(&self, idx: usize, pose: C::P<f64>, update_mode: IKGoalUpdateMode) {
        self.update_function(|x, y| {
            x.ik_goals.update_ik_goal(idx, pose.clone(), update_mode.clone());
            y.ik_goals.update_ik_goal(idx, pose.o3dpose_to_other_generic_category::<E::T, C>(), update_mode.clone());
        });
    }

    #[inline]
    fn update_prev_states(&self, state: Vec<f64>) {
        self.update_function(|x, y| {
            x.prev_states.update(state.clone());
            y.prev_states.update(state.ovec_to_other_ad_type::<E::T>());
            // x.prev_states.update(state.clone());
            // y.prev_states.update(state.ovec_to_other_ad_type::<E::T>());
        });
    }
}

#[serde_as]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct IKGoal<T: AD, P: O3DPose<T>> {
    pub (crate) goal_link_idx: usize,
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    pub (crate) goal_pose: P,
    #[serde_as(as = "SerdeAD<T>")]
    pub (crate) weight: T,
}
impl<T: AD, P: O3DPose<T>> IKGoal<T, P> {
    pub fn new(goal_link_idx: usize, goal_pose: P, weight: T) -> Self {
        Self { goal_link_idx, goal_pose, weight }
    }
    pub fn to_new_ad_type<T1: AD>(&self) -> IKGoal<T1, <P::Category as O3DPoseCategory>::P<T1>> {
        let json_str = self.to_json_string();
        IKGoal::<T1, <P::Category as O3DPoseCategory>::P<T1>>::from_json_string(&json_str)
    }
    pub fn to_new_generic_types<T1: AD, C: O3DPoseCategory>(&self) -> IKGoal<T1, C::P<T1>> {
        let json_str = self.to_json_string();
        IKGoal::<T1, C::P<T1>>::from_json_string(&json_str)
    }
    #[inline]
    pub fn update_goal_pose(&mut self, pose: P, update_mode: IKGoalUpdateMode) {
        match update_mode {
            IKGoalUpdateMode::LocalRelative => {
                self.goal_pose = self.goal_pose.mul(&pose);
            }
            IKGoalUpdateMode::GlobalRelative => {
                self.goal_pose = pose.mul(&self.goal_pose);
            }
            IKGoalUpdateMode::Absolute => {
                self.goal_pose = pose;
            }
        }
    }
    #[inline]
    pub fn update_goal_pose_interpolated(&mut self, pose: &P, fk_res: &FKResult<T, P>, max_translation: T, max_rotation: T) {
        let curr_pose = fk_res.get_link_pose(self.goal_link_idx).as_ref().expect("error");
        let interpolated_pose = curr_pose.interpolate_with_separate_max_translation_and_rotation(pose, max_translation, max_rotation);
        self.update_goal_pose(interpolated_pose, IKGoalUpdateMode::Absolute);
    }
}

#[derive(Clone, Debug)]
pub enum IKGoalUpdateMode {
    LocalRelative, GlobalRelative, Absolute
}

pub trait IKGoalVecTrait<T: AD, P: O3DPose<T>> {
    fn to_other_ad_type<T1: AD>(&self) -> Vec<IKGoal<T1, <P::Category as O3DPoseCategory>::P<T1>>>;
    fn to_other_generic_types<T1: AD, C: O3DPoseCategory>(&self) -> Vec<IKGoal<T1, C::P<T1>>>;
    fn update_ik_goal(&mut self, idx: usize, pose: P, update_mode: IKGoalUpdateMode);
    fn get_ik_goal_pose(&self, idx: usize) -> &P;
}
impl<T: AD, P: O3DPose<T>> IKGoalVecTrait<T, P> for Vec<IKGoal<T, P>> {
    fn to_other_ad_type<T1: AD>(&self) -> Vec<IKGoal<T1, <P::Category as O3DPoseCategory>::P<T1>>> {
        let mut out = vec![];
        self.iter().for_each(|x| { out.push(x.to_new_ad_type::<T1>()); });
        out
    }
    fn to_other_generic_types<T1: AD, C: O3DPoseCategory>(&self) -> Vec<IKGoal<T1, C::P<T1>>> {
        let mut out = vec![];
        self.iter().for_each(|x| { out.push(x.to_new_generic_types::<T1, C>()); });
        out
    }
    #[inline]
    fn update_ik_goal(&mut self, idx: usize, pose: P, update_mode: IKGoalUpdateMode) {
        self[idx].update_goal_pose(pose, update_mode);
    }
    #[inline]
    fn get_ik_goal_pose(&self, idx: usize) -> &P {
        &self[idx].goal_pose
    }
}
pub trait IKGoalRwLockVecTrait<T: AD, P: O3DPose<T>> {
    fn to_other_ad_type<T1: AD>(&self) -> RwLock<Vec<IKGoal<T1, <P::Category as O3DPoseCategory>::P<T1>>>>;
    fn to_other_generic_types<T1: AD, C1: O3DPoseCategory>(&self) -> RwLock<Vec<IKGoal<T1, C1::P<T1>>>>;
    fn update_ik_goal(&self, idx: usize, pose: P, update_mode: IKGoalUpdateMode);
}
impl<T: AD, P: O3DPose<T>> IKGoalRwLockVecTrait<T, P> for RwLock<Vec<IKGoal<T, P>>> {
    fn to_other_ad_type<T1: AD>(&self) -> RwLock<Vec<IKGoal<T1, <P::Category as O3DPoseCategory>::P<T1>>>> {
        self.to_other_generic_types::<T1, P::Category>()
    }

    fn to_other_generic_types<T1: AD, C1: O3DPoseCategory>(&self) -> RwLock<Vec<IKGoal<T1, C1::P<T1>>>> {
        let r = self.read().unwrap();
        RwLock::new(r.to_other_generic_types::<T1, C1>())
    }

    #[inline(always)]
    fn update_ik_goal(&self, idx: usize, pose: P, update_mode: IKGoalUpdateMode) {
        let mut binding = self.write().expect("error");
        binding.update_ik_goal(idx, pose, update_mode);
    }
}

#[derive(Clone, Debug)]
pub struct IKPrevStates<T: AD> {
    pub (crate) prev_state_0: Vec<T>,
    pub (crate) prev_state_1: Vec<T>,
    pub (crate) prev_state_2: Vec<T>,
    pub (crate) prev_state_3: Vec<T>,
}
impl<T: AD> IKPrevStates<T> {
    pub fn new(init_state: Vec<T>) -> Self {
        Self {
            prev_state_0: init_state.clone(),
            prev_state_1: init_state.clone(),
            prev_state_2: init_state.clone(),
            prev_state_3: init_state,
        }
    }
    #[inline(always)]
    pub fn update<T1: AD, V: OVec<T1>>(&mut self, new_state: V) {
        let new_state = new_state.ovec_to_other_generic_category::<T, OVecCategoryVec>();
        self.update_internal(new_state);
    }
    pub fn to_other_ad_type<T1: AD>(&self) -> IKPrevStates<T1> {
        IKPrevStates {
            prev_state_0: self.prev_state_0.ovec_to_other_ad_type::<T1>(),
            prev_state_1: self.prev_state_1.ovec_to_other_ad_type::<T1>(),
            prev_state_2: self.prev_state_2.ovec_to_other_ad_type::<T1>(),
            prev_state_3: self.prev_state_3.ovec_to_other_ad_type::<T1>(),
        }
    }
    #[inline(always)]
    fn update_internal(&mut self, new_state: Vec<T>) {
        self.prev_state_3 = self.prev_state_2.to_owned();
        self.prev_state_2 = self.prev_state_1.to_owned();
        self.prev_state_1 = self.prev_state_0.to_owned();
        self.prev_state_0 = new_state;
    }
}
pub trait IKPrevStatesRwLockTrait {
    fn update<T1: AD, V: OVec<T1>>(&self, new_state: V);
    fn to_other_ad_type<T1: AD>(&self) -> RwLock<IKPrevStates<T1>>;
}
impl<T: AD> IKPrevStatesRwLockTrait for RwLock<IKPrevStates<T>> {
    fn update<T1: AD, V: OVec<T1>>(&self, new_state: V) {
        let mut binding = self.write().unwrap();
        binding.update(new_state);
    }

    fn to_other_ad_type<T1: AD>(&self) -> RwLock<IKPrevStates<T1>> {
        let r = self.read().unwrap();
        RwLock::new(r.to_other_ad_type::<T1>())
    }
}