use std::sync::{Arc, RwLock};
use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::{AliasO3DPoseCategory, O3DPose, O3DPoseCategory};
use optima_3d_spatial::optima_3d_rotation::O3DRotation;
use optima_3d_spatial::optima_3d_vec::{O3DVec, O3DVecCategoryArr, O3DVecCategoryTrait};
use optima_geometry::{pt_dis_to_line};
use optima_linalg::{OLinalgCategory, OVec};
use optima_proximity::pair_group_queries::{OwnedPairGroupQry, OParryFilterOutput, OParryPairSelector, OProximityLossFunction};
use optima_proximity::trait_aliases::{AliasParryGroupFilterQry, AliasParryToProximityQry};
use crate::robot::{FKResult, ORobot};
use crate::robotics_optimization::robotics_optimization_ik::{IKGoal, IKPrevStates};

pub fn robot_self_proximity_refilter_check<'a, T, C, L, FQ>(robot: &ORobot<T, C, L>, filter_query: &OwnedPairGroupQry<'a, T, FQ>, inputs: &[T], fk_res: &FKResult<T, C::P<T>>, last_proximity_filter_state: &Arc<RwLock<Option<Vec<f64>>>>, filter_output: &Arc<RwLock<Option<OParryFilterOutput>>>, linf_dis_cutoff: f64)
    where T: AD,
          C: O3DPoseCategory + 'static,
          L: OLinalgCategory + 'static,
          FQ: AliasParryGroupFilterQry
{
    let inputs_as_vec = inputs.to_vec().o3dvec_to_other_ad_type::<f64>();
    let binding = last_proximity_filter_state.as_ref().read();
    let binding = binding.as_ref().unwrap();
    let dis = match binding.as_ref() {
        None => { f64::MAX }
        Some(state) => {
            state.ovec_sub(&inputs_as_vec).ovec_p_norm(&15.0)
        }
    };

    if dis > linf_dis_cutoff {
        let new_filter = robot.parry_shape_scene_self_query_from_fk_res(fk_res, filter_query, &OParryPairSelector::HalfPairs, false);
        // *filter_output.borrow_mut() = Some(new_filter);
        // *last_proximity_filter_state.borrow_mut() = Some(inputs_as_vec);
        *filter_output.write().unwrap() = Some(new_filter);
        *last_proximity_filter_state.write().unwrap() = Some(inputs_as_vec);
    }
}

pub fn robot_self_proximity_objective<'a, T, C, L, Q>(robot: &ORobot<T, C, L>, fk_res: &FKResult<T, C::P<T>>, distance_query: &OwnedPairGroupQry<'a, T, Q>, selector: &OParryPairSelector, cutoff: T, p_norm: T, loss_function: OProximityLossFunction, freeze: bool) -> T
    where T: AD,
          C: O3DPoseCategory + 'static,
          L: OLinalgCategory + 'static,
          Q: AliasParryToProximityQry
{
    let res = robot.parry_shape_scene_self_query_from_fk_res(fk_res, distance_query, selector, freeze);
    res.get_proximity_objective_value(cutoff, p_norm, loss_function)
}

pub fn robot_ik_goals_objective<'a, T, C>(fk_res: &FKResult<T, C::P<T>>, ik_goals: &Vec<IKGoal<T, C::P<T>>>) -> T
    where T: AD,
          C: O3DPoseCategory + 'static,
{
    let mut out = T::zero();
    if ik_goals.len() == 0 { return out; }

    ik_goals.iter().for_each(|ik_goal| {
        let pose = fk_res.get_link_pose(ik_goal.goal_link_idx).as_ref().expect("error");
        // let interpolated_ik_goal = pose.interpolate_with_max_translation_and_rotation(&ik_goal.goal_pose, max_translation, max_rotation);
        // let dis = pose.dis(&ik_goal.goal_pose);
        // out += ik_goal.weight * dis;
        let position_error = ik_goal.goal_pose.translation().o3dvec_sub(pose.translation()).norm();
        let rotation_error = ik_goal.goal_pose.rotation().dis(&pose.rotation());
        out += ik_goal.weight * (position_error + rotation_error);
    });

    out /= T::constant(ik_goals.len() as f64);

    out
}

pub fn robot_link_look_at_objective<'a, T, C>(fk_res: &FKResult<T, C::P<T>>, looker_link: usize, looker_link_forward_axis: &AxisDirection, look_at_target: &LookAtTarget<T, O3DVecCategoryArr>) -> T
    where T: AD,
          C: O3DPoseCategory + 'static
{
    let looker_pose = fk_res.get_link_pose(looker_link).as_ref().expect("error");
    let looker_location = looker_pose.translation().o3dvec_to_other_generic_category::<T, O3DVecCategoryArr>();
    let axes = looker_pose.rotation().coordinate_frame_vectors();
    let axis = match looker_link_forward_axis {
        AxisDirection::X => { axes[0] }
        AxisDirection::Y => { axes[1] }
        AxisDirection::Z => { axes[2] }
        AxisDirection::NegX => { axes[0].o3dvec_scalar_mul(T::constant(-1.0)) }
        AxisDirection::NegY => { axes[1].o3dvec_scalar_mul(T::constant(-1.0)) }
        AxisDirection::NegZ => { axes[2].o3dvec_scalar_mul(T::constant(-1.0)) }
    };

    let scaled_looker_axis = axis.o3dvec_scalar_mul(T::constant(10.0));
    let cast_out_looker_location = looker_location.o3dvec_add(&scaled_looker_axis.o3dvec_scalar_mul(T::constant(20.0)));

    let look_at_target_location = match look_at_target {
        LookAtTarget::Absolute(position) => { position.o3dvec_to_other_generic_category::<T, O3DVecCategoryArr>() }
        LookAtTarget::RobotLink(idx) => {
            let look_at_target_link = fk_res.get_link_pose(*idx).as_ref().expect("error");
            look_at_target_link.translation().o3dvec_to_other_generic_category::<T, O3DVecCategoryArr>()
        }
        LookAtTarget::PhantomData(_) => { unreachable!() }
    };

    let res = pt_dis_to_line(&look_at_target_location, &looker_location, &cast_out_looker_location, true);

    res.0
}

pub fn robot_link_look_at_roll_prevention_objective<'a, T, C>(fk_res: &FKResult<T, C::P<T>>, looker_link: usize, looker_link_side_axis: &AxisDirection) -> T
    where T: AD,
          C: AliasO3DPoseCategory {
    let looker_link_pose = fk_res.get_link_pose(looker_link).as_ref().expect("error");
    let looker_link_pose_coordinate_frame_vectors = looker_link_pose.rotation().coordinate_frame_vectors();
    let looker_link_side_vector = match looker_link_side_axis {
        AxisDirection::X => { looker_link_pose_coordinate_frame_vectors[0] }
        AxisDirection::Y => { looker_link_pose_coordinate_frame_vectors[1] }
        AxisDirection::Z => { looker_link_pose_coordinate_frame_vectors[2] }
        AxisDirection::NegX => { looker_link_pose_coordinate_frame_vectors[0].o3dvec_scalar_mul(T::constant(-1.0)) }
        AxisDirection::NegY => { looker_link_pose_coordinate_frame_vectors[1].o3dvec_scalar_mul(T::constant(-1.0)) }
        AxisDirection::NegZ => { looker_link_pose_coordinate_frame_vectors[2].o3dvec_scalar_mul(T::constant(-1.0)) }
    };

    let up_vector = [T::zero(), T::zero(), T::one()];
    let res = looker_link_side_vector.o3dvec_dot(&up_vector).powi(2);

    res
}

pub fn robot_goal_distance_between_looker_and_look_at_target_objective<'a, T, C>(fk_res: &FKResult<T, C::P<T>>, goal_distance: T, looker_link: usize, look_at_target: &LookAtTarget<T, O3DVecCategoryArr>) -> T
    where T: AD, C: AliasO3DPoseCategory
{
    let looker_pose = fk_res.get_link_pose(looker_link).as_ref().expect("error");
    let looker_location = looker_pose.translation().o3dvec_to_other_generic_category::<T, O3DVecCategoryArr>();
    let look_at_target_location = match look_at_target {
        LookAtTarget::Absolute(position) => { position.o3dvec_to_other_generic_category::<T, O3DVecCategoryArr>() }
        LookAtTarget::RobotLink(idx) => {
            let look_at_target_link = fk_res.get_link_pose(*idx).as_ref().expect("error");
            look_at_target_link.translation().o3dvec_to_other_generic_category::<T, O3DVecCategoryArr>()
        }
        LookAtTarget::PhantomData(_) => { unreachable!() }
    };

    let dis = looker_location.o3dvec_sub(&look_at_target_location).norm();
    let diff = dis - goal_distance;
    let diff_squared = diff.powi(2);

    diff_squared
}

pub fn robot_per_instant_velocity_acceleration_and_jerk_objectives<T: AD>(inputs: &[T], prev_states: &IKPrevStates<T>, p_norm: T) -> (T, T, T) {
    per_instant_velocity_acceleration_and_jerk_objectives(inputs, &prev_states.prev_state_0, &prev_states.prev_state_1, &prev_states.prev_state_2, p_norm)
}

pub fn per_instant_velocity_acceleration_and_jerk_objectives<T: AD, V: OVec<T>>(inputs: &[T], prev_state_0: &V, prev_state_1: &V, prev_state_2: &V, p_norm: T) -> (T, T, T) {
    let x_vec = V::ovec_from_slice(inputs);
    let v0 = x_vec.ovec_sub(prev_state_0);
    let v1 = prev_state_0.ovec_sub(prev_state_1);
    let v2 = prev_state_1.ovec_sub(prev_state_2);
    let a0 = v0.ovec_sub(&v1);
    let a1 = v1.ovec_sub(&v2);
    let j0 = a0.ovec_sub(&a1);

    let v = v0.ovec_p_norm(&p_norm);
    let a = a0.ovec_p_norm(&p_norm);
    let j = j0.ovec_p_norm(&p_norm);

    (v,a,j)
}

pub fn min_velocity_over_path_objective<T: AD, V: OVec<T>>(inputs: &Vec<V>, p_norm: T) -> T {
    let mut v_vec = vec![];
    let num_inputs = inputs.len();
    for i in 0..num_inputs - 1 {
        let a = &inputs[i];
        let b = &inputs[i+1];
        let v = a.ovec_sub(b).ovec_p_norm(&p_norm);
        v_vec.push(v);
    }

    return v_vec.ovec_p_norm(&p_norm);
}

pub fn min_acceleration_over_path_objective<T: AD, V: OVec<T>>(inputs: &Vec<V>, p_norm: T) -> T {
    let mut a_vec = vec![];
    let num_inputs = inputs.len();
    for i in 0..num_inputs - 2 {
        let a = &inputs[i];
        let b = &inputs[i+1];
        let c = &inputs[i+2];
        let v1 = a.ovec_sub(b);
        let v2 = b.ovec_sub(c);
        let acc = v1.ovec_sub(&v2).ovec_p_norm(&p_norm);
        a_vec.push(acc);
    }

    return a_vec.ovec_p_norm(&p_norm);
}

pub fn min_jerk_over_path_objective<T: AD, V: OVec<T>>(inputs: &Vec<V>, p_norm: T) -> T {
    let mut j_vec = vec![];
    let num_inputs = inputs.len();
    for i in 0..num_inputs - 3 {
        let a = &inputs[i];
        let b = &inputs[i+1];
        let c = &inputs[i+2];
        let d = &inputs[i+3];
        let v1 = a.ovec_sub(b);
        let v2 = b.ovec_sub(c);
        let v3 = c.ovec_sub(d);
        let a1 = v1.ovec_sub(&v2);
        let a2 = v2.ovec_sub(&v3);
        let j = a1.ovec_sub(&a2).ovec_p_norm(&p_norm);
        j_vec.push(j);
    }

    return j_vec.ovec_p_norm(&p_norm);
}

#[derive(Clone, Debug)]
pub enum AxisDirection { X, Y, Z, NegX, NegY, NegZ }

#[derive(Clone, Debug)]
pub enum LookAtTarget<T: AD, VC: O3DVecCategoryTrait> {
    Absolute(VC::V<T>),
    RobotLink(usize),
    PhantomData(T)
}
impl<T: AD, VC: O3DVecCategoryTrait> LookAtTarget<T, VC> {
    pub fn to_other_ad_type<T1: AD>(&self) -> LookAtTarget<T1, VC> {
        self.to_other_generic_types::<T1, VC>()
    }
    pub fn to_other_generic_types<T1: AD, VC1: O3DVecCategoryTrait>(&self) -> LookAtTarget<T1, VC1> {
        match self {
            LookAtTarget::Absolute(position) => { LookAtTarget::<T1, VC1>::Absolute(position.o3dvec_to_other_generic_category::<T1, VC1>()) }
            LookAtTarget::RobotLink(idx) => { LookAtTarget::RobotLink(*idx) }
            LookAtTarget::PhantomData(t1) => { LookAtTarget::PhantomData(t1.to_other_ad_type::<T1>()) }
        }
    }
}

pub trait LookAtTargetRwLockTrait {
    fn to_other_generic_types<T1: AD, VC1: O3DVecCategoryTrait>(&self) -> RwLock<LookAtTarget<T1, VC1>>;
}
impl<T: AD, VC: O3DVecCategoryTrait> LookAtTargetRwLockTrait for RwLock<LookAtTarget<T, VC>> {
    fn to_other_generic_types<T1: AD, VC1: O3DVecCategoryTrait>(&self) -> RwLock<LookAtTarget<T1, VC1>> {
        let r = self.read().unwrap();
        RwLock::new(r.to_other_generic_types::<T1, VC1>())
    }
}


