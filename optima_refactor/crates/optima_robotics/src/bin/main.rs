use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::ImplicitDualQuaternion;
use optima_file::traits::ToJsonString;
use optima_linalg::vecs_and_mats::NalgebraLinalg;
use optima_robotics::robot::{ORobot, ORobotDefault};
use optima_robotics::robotics_traits::OForwardKinematicsTrait;

fn main() {
    let r = ORobot::<f32, ImplicitDualQuaternion<_>, NalgebraLinalg>::from_urdf("ur5");
    let res = r.forward_kinematics(&[1.; 6], None);
    println!("{:?}", res);
}