use std::os::raw::*;
use ad_trait::differentiable_function::ForwardADMulti2;
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::{Isometry3, Quaternion, UnitQuaternion, Vector3};
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryIsometry3};
use optima_linalg::OLinalgCategoryNalgebra;
use optima_optimization2::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization2::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{OwnedParryDistanceGroupSequenceFilter, ParryDistanceGroupSequenceFilter, ParryDistanceGroupSequenceFilterArgs, ProximityLossFunction};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity::proxima::{OwnedParryProximaAsProximityQry, PairGroupQryArgsParryProxima, ParryProximaAsProximityQry, ProximaTermination};
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_optimization2::robotics_optimization_ik::{DifferentiableBlockIKObjective, DifferentiableBlockIKObjectiveTrait, IKGoalUpdateMode};

type FAD = adfn<8>;

#[no_mangle]
pub unsafe extern "C" fn get_default_robot(robot_name: *const c_char) -> *const ORobotDefault {
    let c_str = std::ffi::CStr::from_ptr(robot_name);
    let s = c_str.to_str().expect("Not a valid UTF-8 string");
    let r = ORobotDefault::load_from_saved_robot(s);
    Box::into_raw(Box::new(r))
}

#[no_mangle]
pub unsafe extern "C" fn get_default_ik_differentiable_block<'a>(robot: *const ORobotDefault, goal_link_idx: *const c_int, init_state: *const c_double, joint_state_length: c_int) -> *const DifferentiableBlockIKObjective<'a, O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra, ParryDistanceGroupSequenceFilter, ParryProximaAsProximityQry, ForwardADMulti2<FAD>> {
    let x_slice: &[c_double] = std::slice::from_raw_parts(init_state, joint_state_length as usize);
    let x = x_slice.to_vec();
    let goal_link_idx = goal_link_idx as usize;

    let fq = OwnedParryDistanceGroupSequenceFilter::new(ParryDistanceGroupSequenceFilterArgs::new(vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB, ParryShapeRep::Full], vec![], 0.6, true, ParryDisMode::ContactDis));
    let q = OwnedParryProximaAsProximityQry::new(PairGroupQryArgsParryProxima::new(ParryShapeRep::Full, true, false, ProximaTermination::MaxError(0.15), ProximityLossFunction::Hinge, 15.0, 0.6));
    // let q = OwnedParryDistanceAsProximityGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, true, false, -1000.0, false));
    let db = robot.as_ref().unwrap().get_ik_differentiable_block(ForwardADMulti2::<FAD>::new(), fq, q, None, &x, vec![goal_link_idx], 0.09, 0.6, 1.0, 0.1, 1.0, 0.3, 0.1);

    Box::into_raw(Box::new(db))
}

#[no_mangle]
pub unsafe extern "C" fn get_default_ik_optimizer(robot: *const ORobotDefault) -> *const SimpleOpEnOptimizer {
    let r = robot.as_ref().unwrap();
    let o = SimpleOpEnOptimizer::new(r.get_dof_lower_bounds(), r.get_dof_upper_bounds(), 0.001);
    Box::into_raw(Box::new(o))
}

/// new_ee_orientation should be list of four values, a unit quaternion in format [w x y z]
#[no_mangle]
pub unsafe extern "C" fn update_ik_differentiable_block(new_ee_position: *const c_double, new_ee_orientation: *const c_double, previous_solution: *const c_double, joint_state_length: c_int, differentiable_block: *const DifferentiableBlockIKObjective<O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra, ParryDistanceGroupSequenceFilter, ParryProximaAsProximityQry, ForwardADMulti2<FAD>>, ) {
    let position_slice: &[c_double] = std::slice::from_raw_parts(new_ee_position, 3);
    let position = position_slice.to_vec();
    let pos: Vector3<f64> = Vector3::new(position[0], position[1], position[2]);
    let orientation_slice: &[c_double] = std::slice::from_raw_parts(new_ee_orientation, 4);
    let orientation = orientation_slice.to_vec();
    let quat: UnitQuaternion<f64> = UnitQuaternion::from_quaternion(Quaternion::new(orientation[0], orientation[1], orientation[2], orientation[3]));
    let previous_solution_slice: &[c_double] = std::slice::from_raw_parts(previous_solution, joint_state_length as usize);
    let previous_solution = previous_solution_slice.to_vec();

    differentiable_block.update_ik_pose(0, Isometry3::from_translation_and_rotation(&pos, &quat), IKGoalUpdateMode::Absolute);
    differentiable_block.update_prev_states(previous_solution);
}

#[no_mangle]
pub unsafe extern "C" fn ik_optimize(init_condition: *const c_double, joint_state_length: c_int, differentiable_block: *const DifferentiableBlockIKObjective<O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra, ParryDistanceGroupSequenceFilter, ParryProximaAsProximityQry, ForwardADMulti2<FAD>>, optimizer: *const SimpleOpEnOptimizer) -> IKOptResult {
    let x_slice: &[c_double] = std::slice::from_raw_parts(init_condition, joint_state_length as usize);
    let x = x_slice.to_vec();
    let o = optimizer.as_ref().unwrap();
    let db = differentiable_block.as_ref().unwrap();
    let res = o.optimize_unconstrained(&x, db);
    let solution = res.x_star().to_vec();
    let l = solution.len();
    let ptr = solution.as_ptr();

    IKOptResult { data: ptr, length: l as c_int }
}

#[repr(C)]
pub struct IKOptResult {
    pub data: *const c_double,
    pub length: c_int,
}