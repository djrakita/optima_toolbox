use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryImplicitDualQuaternion};
use optima_linalg::OLinalgCategoryNDarray;
use optima_robotics::chain::{OChain, OChainDefault};
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};

fn main() {
    let b1_chain = OChainDefault::from_urdf("b1");
    let z1_chain = OChainDefault::from_urdf("z1");

    let mut robot = ORobotDefault::new_empty();
    robot.add_chain(b1_chain, 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    robot.add_chain(z1_chain, 1, 0, &Isometry3::from_constructors(&[0.1, 0.0, 0.02], &[0.0; 3]), [0.0; 3], OJointType::Fixed, OJointLimit::default());
}