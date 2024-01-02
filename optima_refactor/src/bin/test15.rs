use optima_wrappers::ffi_wrappers::ik_solvers2::compute_interpolated_motion_path_to_ee_pose;
use optima_wrappers::ffi_wrappers::set_global_robot;

fn main() {
    set_global_robot("xarm7_with_gripper_and_rail_8dof");

    let res = compute_interpolated_motion_path_to_ee_pose(19, vec![0.3, 0.3, 0.3], vec![1.0, 0.,0.,0.], vec![0.0; 8]);
    println!("{:?}", res.len());
}