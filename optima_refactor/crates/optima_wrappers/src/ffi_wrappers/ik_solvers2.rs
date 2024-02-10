use std::ffi::{c_double, c_int};
use std::sync::{OnceLock};
use ad_trait::differentiable_function::ForwardADMulti;
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryIsometry3};
use optima_3d_spatial::optima_3d_rotation::QuatConstructor;
use optima_interpolation::InterpolatorTrait;
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_linalg::OLinalgCategoryNalgebra;
use optima_optimization::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{EmptyParryFilter, EmptyToParryProximity, OwnedEmptyParryFilter, OwnedEmptyToProximityQry};
use optima_robotics::robotics_optimization::robotics_optimization_ik::{DifferentiableBlockIKObjective, DifferentiableBlockIKObjectiveTrait, IKGoalUpdateMode};
use crate::ffi_wrappers::{DoubleArray, ArrayOfDoubleArrays, FFIConverters, GLOBAL_ROBOT};

type FAD = adfn<8>;

thread_local! {
    static GLOBAL_STATIC_IK_DB: OnceLock<DifferentiableBlockIKObjective<O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra, EmptyParryFilter, EmptyToParryProximity, ForwardADMulti<FAD>>> = OnceLock::new();
    static GLOBAL_IK_OPTIMIZER: OnceLock<SimpleOpEnOptimizer> = OnceLock::new();
}

pub fn compute_interpolated_motion_path_to_ee_pose(goal_link_idx: usize, ee_position: Vec<f64>, ee_orientation: Vec<f64>, init_state: Vec<f64>) -> Vec<Vec<f64>> {
    let r = GLOBAL_ROBOT.get_or_init(|| panic!("use set_global_robot to initialize robot"));

    let res = GLOBAL_STATIC_IK_DB.with(|once_lock_ik_diff_block| {
        let res = GLOBAL_IK_OPTIMIZER.with(|once_lock_ik_optimizer| {
            let db = once_lock_ik_diff_block.get_or_init(|| r.get_ik_differentiable_block(ForwardADMulti::new(), OwnedEmptyParryFilter::new(()), OwnedEmptyToProximityQry::new(()), None, &[0.0; 8], vec![goal_link_idx], 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0));

            db.update_ik_pose(0, Isometry3::from_constructors(&ee_position, &QuatConstructor::new_from_wxyz_ovec(&ee_orientation)), IKGoalUpdateMode::Absolute);

            let o = once_lock_ik_optimizer.get_or_init(|| SimpleOpEnOptimizer::new(r.get_dof_lower_bounds(), r.get_dof_upper_bounds(), 0.001));

            let r = o.optimize_unconstrained(&init_state, &db);
            let solution_point = r.x_star().to_vec();

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


