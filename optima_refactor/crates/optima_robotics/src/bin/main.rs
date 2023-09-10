use ad_trait::differentiable_function::{ForwardAD, ForwardADMulti};
use ad_trait::forward_ad::adf::{adf_f32x16, adf_f32x2, adf_f32x8};
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optimization_engine::panoc::PANOCCache;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryIsometry3};
use optima_linalg::NalgebraLinalg;
use optima_optimization::derivative_based_optimization::DerivBasedOptSolver;
use optima_optimization::derivative_based_optimization::optimization_engine::{OpEnSimple, OpEnSimpleArgs};
use optima_robotics::robot::{ORobotDefault};
use optima_robotics::robotics_optimization_solvers::{SimpleIKArgs, SimpleIKFunction};

fn main() {
    let robot1 = ORobotDefault::<f64>::new_from_single_chain_name("ur5");
    let robot2 = robot1.to_new_generic_types_default::<adf_f32x2>();
    let goal_pose1 = Isometry3::from_constructors(&[0.1,0.,0.7], &[0.,0.,0.]);
    let goal_pose2 = goal_pose1.to_other_ad_type::<adf_f32x2>();

    let mut ik = DerivBasedOptSolver::<SimpleIKFunction<O3DPoseCategoryIsometry3, NalgebraLinalg>, ForwardADMulti<adf_f32x2>, OpEnSimple>::new(
        SimpleIKArgs::new(robot1, 1, 7, goal_pose1),
        SimpleIKArgs::new(robot2, 1, 7, goal_pose2),
        (),
        OpEnSimpleArgs::new(PANOCCache::new(6, 1e-5, 3), vec![0.001; 6])
    );

    let res = ik.optimize();
    println!("{:?}", res);
}