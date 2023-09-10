use ad_trait::differentiable_function::{DerivativeMethodTrait, FiniteDifferencing, ForwardAD, ForwardADMulti, ReverseAD};
use ad_trait::forward_ad::adf::{adf_f32x16, adf_f32x2, adf_f32x4, adf_f32x8, adf_f64x2, adf_f64x4, adf_f64x8};
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optimization_engine::panoc::PANOCCache;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryImplicitDualQuaternion, O3DPoseCategoryIsometry3, O3DPoseCategoryTrait};
use optima_linalg::{OLinalgCategoryNalgebra, OLinalgCategoryNDarray};
use optima_optimization::derivative_based_optimization::DerivBasedOptSolver;
use optima_optimization::derivative_based_optimization::optimization_engine::{OpEnSimple, OpEnSimpleArgs};
use optima_robotics::robot::{ORobot, ORobotDefault};
use optima_robotics::robotics_optimization_solvers::{SimpleIKArgs, SimpleIKFunction, SimpleIKSolver};

type DerivativeMethod = ForwardADMulti<adf_f32x8>;
type DerivativeT = <DerivativeMethod as DerivativeMethodTrait>::T;
type LinalgCategory = OLinalgCategoryNDarray;
type PoseCategory = O3DPoseCategoryImplicitDualQuaternion;
type PoseType = <PoseCategory as O3DPoseCategoryTrait>::P<f64>;

fn main() {
    let robot1 = ORobot::<f64, PoseCategory, LinalgCategory>::new_from_single_chain_name("ur5");
    let robot2 = robot1.to_new_ad_type::<DerivativeT>();
    let goal_pose1 = PoseType::from_constructors(&[0.1,0.,0.7], &[0.,0.,0.]);
    let goal_pose2 = goal_pose1.to_other_ad_type::<DerivativeT>();

    let mut ik = DerivBasedOptSolver::<SimpleIKFunction<PoseCategory, LinalgCategory>, DerivativeMethod, OpEnSimple>::new(
        SimpleIKArgs::new(robot1, 1, 9, goal_pose1),
        SimpleIKArgs::new(robot2, 1, 9, goal_pose2),
        (),
        OpEnSimpleArgs::new(PANOCCache::new(6, 1e-3, 3), vec![0.001; 6])
    );

    let res = ik.optimize();
    println!("{:?}", res);

    let res = ik.optimize();
    println!("{:?}", res);

    let res = ik.optimize();
    println!("{:?}", res);

    let res = ik.optimize();
    println!("{:?}", res);
}