use std::ffi::{c_double, c_int};
use std::os::raw::c_char;
use std::sync::{OnceLock};
use std::thread::LocalKey;
use ad_trait::differentiable_function::{FiniteDifferencing};
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryIsometry3};
use optima_3d_spatial::optima_3d_rotation::QuatConstructor;
use optima_interpolation::InterpolatorTrait;
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_linalg::OLinalgCategoryNalgebra;
use optima_optimization::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{EmptyParryFilter, EmptyToParryProximity, OwnedEmptyParryFilter, OwnedEmptyToProximityQry};
use optima_robotics::robotics_optimization::robotics_optimization_functions::{AxisDirection, LookAtTarget};
use optima_robotics::robotics_optimization::robotics_optimization_ik::{DifferentiableBlockIKObjective, DifferentiableBlockIKObjectiveTrait, IKGoalUpdateMode};
use optima_robotics::robotics_optimization::robotics_optimization_look_at::{DifferentiableBlockLookAt, DifferentiableBlockLookAtTrait};
use crate::ffi_wrappers::{DoubleArray, ArrayOfDoubleArrays, FFIConverters, GLOBAL_ROBOT};

// type FAD = adfn<8>;

thread_local! {
    static GLOBAL_STATIC_IK_DB: OnceLock<DifferentiableBlockIKObjective<O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra, EmptyParryFilter, EmptyToParryProximity, FiniteDifferencing>> = OnceLock::new();
    static GLOBAL_STATIC_VIEWPOINT_DB: OnceLock<DifferentiableBlockLookAt<FiniteDifferencing,O3DPoseCategoryIsometry3,OLinalgCategoryNalgebra,EmptyParryFilter,EmptyToParryProximity>> = OnceLock::new();
    static GLOBAL_IK_OPTIMIZER: OnceLock<SimpleOpEnOptimizer> = OnceLock::new();
}

pub fn compute_interpolated_motion_path_viewpoint(ik_goal_link_idx: usize, ee_position: Vec<f64>, ee_orientation: Vec<f64>, init_state: Vec<f64>, looker_link: usize, looker_forward_axis: AxisDirection, looker_side_axis: AxisDirection, looker_link_position_delta: [f64; 3], looker_link_in_out_delta: f64) -> Vec<Vec<f64>> {
    //let r = GLOBAL_ROBOT.get_or_init(|| panic!("use set_global_robot to initialize robot"));
    let mut robot_lock = GLOBAL_ROBOT.lock().unwrap();
    if robot_lock.is_none() {
        panic!("Use set_global_robot to initialize the robot");
    }
    let r = robot_lock.as_ref().unwrap();

    let res = GLOBAL_STATIC_VIEWPOINT_DB.with(|once_lock_viewpoint_diff_block| {
        let res = GLOBAL_IK_OPTIMIZER.with(|once_lock_ik_optimizer| {
            let db = once_lock_viewpoint_diff_block.get_or_init(|| r.get_default_ik_lookat_differentiable_block(FiniteDifferencing::new(), &init_state, vec![ik_goal_link_idx], looker_link, looker_forward_axis, looker_side_axis, LookAtTarget::RobotLink(ik_goal_link_idx)));

            db.update_ik_pose(0, Isometry3::from_constructors(&ee_position, &QuatConstructor::new_from_wxyz_ovec(&ee_orientation)), IKGoalUpdateMode::Absolute);
            db.move_looker_position_goal_along_direction(looker_link_position_delta);
            db.move_looker_position_goal_in_or_out(looker_link_in_out_delta);

            let o = once_lock_ik_optimizer.get_or_init(|| SimpleOpEnOptimizer::new(r.get_dof_lower_bounds(), r.get_dof_upper_bounds(), 0.001));

            let r = o.optimize_unconstrained(&init_state, &db);
            let solution_point = r.x_star().to_vec();

            db.update_prev_states(solution_point.clone());

            let spline = InterpolatingSpline::new(vec![init_state.clone(), solution_point.clone()], InterpolatingSplineType::Linear)
                .to_arclength_parameterized_interpolator(40);

            let path = spline.interpolate_points_by_arclength_absolute_stride(0.05);
            path
        });
        res
    });

    res
}

pub fn compute_interpolated_motion_path_to_ee_pose(goal_link_idx: usize, ee_position: Vec<f64>, ee_orientation: Vec<f64>, init_state: Vec<f64>) -> Vec<Vec<f64>> {
    //let r = GLOBAL_ROBOT.get_or_init(|| panic!("use set_global_robot to initialize robot"));
    let mut robot_lock = GLOBAL_ROBOT.lock().unwrap();
    if robot_lock.is_none() {
        panic!("Use set_global_robot to initialize the robot");
    }
    let r = robot_lock.as_ref().unwrap();

    let res = GLOBAL_STATIC_IK_DB.with(|once_lock_ik_diff_block| {
        let res = GLOBAL_IK_OPTIMIZER.with(|once_lock_ik_optimizer| {
            let db = once_lock_ik_diff_block.get_or_init(|| r.get_ik_differentiable_block(FiniteDifferencing::new(), OwnedEmptyParryFilter::new(()), OwnedEmptyToProximityQry::new(()), None, &init_state, vec![goal_link_idx], 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0));

            db.update_ik_pose(0, Isometry3::from_constructors(&ee_position, &QuatConstructor::new_from_wxyz_ovec(&ee_orientation)), IKGoalUpdateMode::Absolute);

            let o = once_lock_ik_optimizer.get_or_init(|| SimpleOpEnOptimizer::new(r.get_dof_lower_bounds(), r.get_dof_upper_bounds(), 0.001));

            let r = o.optimize_unconstrained(&init_state, &db);
            let solution_point = r.x_star().to_vec();

            db.update_prev_states(solution_point.clone());

            let spline = InterpolatingSpline::new(vec![init_state.clone(), solution_point.clone()], InterpolatingSplineType::Linear)
                .to_arclength_parameterized_interpolator(40);

            let path = spline.interpolate_points_by_arclength_absolute_stride(0.05);
            path
        });
        res
    });

    res
}

#[no_mangle]
pub unsafe extern "C" fn ffi_compute_interpolated_motion_path_viewpoint(goal_link_idx: c_int, ee_position: *const c_double, ee_orientation: *const c_double, init_state: *const c_double, state_length: c_int, looker_link: c_int, looker_forward_axis: *const c_char, looker_side_axis: *const c_char, looker_link_position_delta: *const c_double, looker_link_in_out_delta: c_double) -> ArrayOfDoubleArrays {
    let ik_goal_link_idx = goal_link_idx as usize;
    let ee_position = FFIConverters::c_double_arr_to_rust_double_vec(ee_position, 3);
    let ee_orientation = FFIConverters::c_double_arr_to_rust_double_vec(ee_orientation, 4);
    let init_state = FFIConverters::c_double_arr_to_rust_double_vec(init_state, state_length);
    let looker_link = FFIConverters::c_int_to_rust_usize(looker_link);
    let looker_forward_axis_string = FFIConverters::c_str_to_rust_string(looker_forward_axis);
    let looker_forward_axis = if looker_forward_axis_string == "X" {
        AxisDirection::X
    } else if looker_forward_axis_string == "Y" {
        AxisDirection::Y
    } else if looker_forward_axis_string == "Z" {
        AxisDirection::Z
    } else { panic!("unsupported") };
    let looker_side_axis_string = FFIConverters::c_str_to_rust_string(looker_side_axis);
    let looker_side_axis = if looker_side_axis_string == "X" {
        AxisDirection::X
    } else if looker_side_axis_string == "Y" {
        AxisDirection::Y
    } else if looker_side_axis_string == "Z" {
        AxisDirection::Z
    } else { panic!("unsupported") };
    let looker_link_position_delta = FFIConverters::c_double_arr_to_rust_double_vec(looker_link_position_delta, 3);
    let looker_link_position_delta = [looker_link_position_delta[0], looker_link_position_delta[1], looker_link_position_delta[2]];
    let res = compute_interpolated_motion_path_viewpoint(ik_goal_link_idx, ee_position, ee_orientation, init_state, looker_link, looker_forward_axis, looker_side_axis, looker_link_position_delta, looker_link_in_out_delta);

    FFIConverters::rust_vec_of_f64_vecs_to_array_of_double_arrays(res)
}

#[no_mangle]
pub unsafe extern "C" fn ffi_compute_interpolated_motion_path_to_ee_pose(goal_link_idx: c_int, ee_position: *const c_double, ee_orientation: *const c_double, init_state: *const c_double, state_length: c_int) -> ArrayOfDoubleArrays {
    let goal_link_idx = goal_link_idx as usize;
    let ee_position = FFIConverters::c_double_arr_to_rust_double_vec(ee_position, 3);
    let ee_orientation = FFIConverters::c_double_arr_to_rust_double_vec(ee_orientation, 4);
    let init_state = FFIConverters::c_double_arr_to_rust_double_vec(init_state, state_length);
    let res = compute_interpolated_motion_path_to_ee_pose(goal_link_idx, ee_position, ee_orientation, init_state);

    FFIConverters::rust_vec_of_f64_vecs_to_array_of_double_arrays(res)
}

#[no_mangle]
pub extern "C" fn ffi_free_double_array(ptr: *mut DoubleArray) {
    unsafe {
        if !ptr.is_null() {
            let _ = Box::from_raw(ptr);
        }
    }
}
#[no_mangle]
pub extern "C" fn ffi_free_array_of_double_arrays(ptr: *mut ArrayOfDoubleArrays) {
    unsafe {
        if !ptr.is_null() {
            let _ = Box::from_raw(ptr);
        }
    }
}

pub fn clear_global_statics(ik_goal_link_idx: usize, init_state: Vec<f64>, looker_link: usize, looker_forward_axis: AxisDirection, looker_side_axis: AxisDirection)
{
    let mut robot_lock = GLOBAL_ROBOT.lock().unwrap();
    if robot_lock.is_none() {
        panic!("Use set_global_robot to initialize the robot");
    }
    let r = robot_lock.as_ref().unwrap();

    let db = r.get_default_ik_lookat_differentiable_block(FiniteDifferencing::new(), &init_state, vec![ik_goal_link_idx], looker_link, looker_forward_axis, looker_side_axis, LookAtTarget::RobotLink(ik_goal_link_idx));
    let o = SimpleOpEnOptimizer::new(r.get_dof_lower_bounds(), r.get_dof_upper_bounds(), 0.001);

    GLOBAL_STATIC_VIEWPOINT_DB.with(|once_lock_viewpoint_diff_block| {
        let _ = once_lock_viewpoint_diff_block.set(db);
    });

    GLOBAL_IK_OPTIMIZER.with(|once_lock_ik_optimizer| {
        let _ = once_lock_ik_optimizer.set(o);
    });
}

#[no_mangle]
pub unsafe extern "C" fn ffi_clear_global_statics(goal_link_idx: c_int, init_state: *const c_double, state_length: c_int, looker_link: c_int, looker_forward_axis: *const c_char, looker_side_axis: *const c_char) {
    let ik_goal_link_idx = goal_link_idx as usize;

    let init_state = FFIConverters::c_double_arr_to_rust_double_vec(init_state, state_length);
    let looker_link = FFIConverters::c_int_to_rust_usize(looker_link);
    let looker_forward_axis_string = FFIConverters::c_str_to_rust_string(looker_forward_axis);
    let looker_forward_axis = if looker_forward_axis_string == "X" {
        AxisDirection::X
    } else if looker_forward_axis_string == "Y" {
        AxisDirection::Y
    } else if looker_forward_axis_string == "Z" {
        AxisDirection::Z
    } else { panic!("unsupported") };
    let looker_side_axis_string = FFIConverters::c_str_to_rust_string(looker_side_axis);
    let looker_side_axis = if looker_side_axis_string == "X" {
        AxisDirection::X
    } else if looker_side_axis_string == "Y" {
        AxisDirection::Y
    } else if looker_side_axis_string == "Z" {
        AxisDirection::Z
    } else { panic!("unsupported") };
    clear_global_statics(ik_goal_link_idx, init_state, looker_link, looker_forward_axis, looker_side_axis);
}


