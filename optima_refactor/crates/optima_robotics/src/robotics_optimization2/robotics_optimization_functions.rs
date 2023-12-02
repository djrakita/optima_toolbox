use std::cell::RefCell;
use std::rc::Rc;
use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategory};
use optima_3d_spatial::optima_3d_vec::O3DVec;
use optima_linalg::{OLinalgCategory, OVec};
use optima_proximity::pair_group_queries::{OPairGroupQryTrait, OwnedPairGroupQry, PairGroupQryOutputCategoryParryDistance, PairGroupQryOutputCategoryParryFilter, ParryFilterOutput, ParryPairSelector, ProximityLossFunctionTrait, ToParryProximityOutputTrait};
use optima_proximity::shapes::ShapeCategoryOParryShape;
use crate::robot::{FKResult, ORobot};
use crate::robotics_optimization2::robotics_optimization_ik::{IKGoal, IKPrevStates};

pub fn robot_self_proximity_refilter_check<'a, T, C, L, FQ>(robot: &ORobot<T, C, L>, filter_query: &OwnedPairGroupQry<'a, T, FQ>, inputs: &[T], fk_res: &FKResult<T, C::P<T>>, last_proximity_filter_state: &Rc<RefCell<Option<Vec<f64>>>>, filter_output: &Rc<RefCell<Option<ParryFilterOutput>>>, linf_dis_cutoff: f64)
    where T: AD,
          C: O3DPoseCategory + 'static,
          L: OLinalgCategory + 'static,
          FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>
{
    let inputs_as_vec = inputs.to_vec().o3dvec_to_other_ad_type::<f64>();
    let dis = match last_proximity_filter_state.as_ref().borrow().as_ref() {
        None => { f64::MAX }
        Some(state) => { state.ovec_sub(&inputs_as_vec).ovec_linf_norm() }
    };

    if dis > linf_dis_cutoff {
        let new_filter = robot.parry_shape_scene_self_query_from_fk_res(fk_res, filter_query, &ParryPairSelector::HalfPairs);
        *filter_output.borrow_mut() = Some(new_filter);
        *last_proximity_filter_state.borrow_mut() = Some(inputs_as_vec);
    }
}

pub fn robot_self_proximity_objective<'a, T, C, L, Q, LF>(robot: &ORobot<T, C, L>, fk_res: &FKResult<T, C::P<T>>, distance_query: &OwnedPairGroupQry<'a, T, Q>, filter_output: &ParryFilterOutput, cutoff: T, p_norm: T, loss_function: LF) -> T
    where T: AD,
          C: O3DPoseCategory + 'static,
          L: OLinalgCategory + 'static,
          Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance>,
          LF: ProximityLossFunctionTrait<T>
{
    let res = robot.parry_shape_scene_self_query_from_fk_res(fk_res, distance_query, filter_output.selector());
    res.compute_proximity_objective_value(cutoff, p_norm, loss_function)
}

pub fn robot_ik_goals_objective<'a, T, C>(fk_res: &FKResult<T, C::P<T>>, ik_goals: &Vec<IKGoal<T, C::P<T>>>) -> T
    where T: AD,
          C: O3DPoseCategory + 'static,
{
    let mut out = T::zero();
    ik_goals.iter().for_each(|ik_goal| {
        let pose = fk_res.get_link_pose(ik_goal.goal_link_idx).as_ref().expect("error");
        let dis = pose.dis(&ik_goal.goal_pose);
        out += ik_goal.weight * dis;
    });

    out /= T::constant(ik_goals.len() as f64);

    out
}

pub fn robot_per_instant_velocity_acceleration_and_jerk_objectives<T: AD>(inputs: &[T], prev_states: &IKPrevStates<T>, p_norm: T) -> (T, T, T) {
    let x_vec = inputs.to_vec();
    let v0 = x_vec.ovec_sub(&prev_states.prev_state_0);
    let v1 = prev_states.prev_state_0.ovec_sub(&prev_states.prev_state_1);
    let v2 = prev_states.prev_state_1.ovec_sub(&prev_states.prev_state_2);
    let a0 = v0.ovec_sub(&v1);
    let a1 = v1.ovec_sub(&v2);
    let j0 = a0.ovec_sub(&a1);

    let v = v0.ovec_p_norm(&p_norm);
    let a = a0.ovec_p_norm(&p_norm);
    let j = j0.ovec_p_norm(&p_norm);

    (v,a,j)
}
