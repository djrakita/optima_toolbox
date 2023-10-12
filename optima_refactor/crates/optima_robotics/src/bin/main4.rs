use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryImplicitDualQuaternion};
use optima_linalg::OLinalgCategoryNDarray;
use optima_robotics::robot::{ORobot, ORobotDefault};
use optima_robotics::robot_set::ORobotSetDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};

fn main() {
    let b1_chain = ORobotDefault::from_urdf("b1");
    let z1_chain = ORobotDefault::from_urdf("z1");

    let mut robot = ORobotSetDefault::new_empty();
    robot.add_robot(b1_chain, 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    robot.add_robot(z1_chain, 1, 0, &Isometry3::from_constructors(&[0.1, 0.0, 0.02], &[0.0; 3]), [0.0; 3], OJointType::Fixed, OJointLimit::default());
}