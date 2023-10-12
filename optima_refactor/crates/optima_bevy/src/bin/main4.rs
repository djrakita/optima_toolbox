use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::{O3DPose};
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::{ORobotDefault};
use optima_robotics::robot_set::ORobotSetDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};
use optima_robotics::robotics_traits::AsRobotTrait;

fn main() {
    let b1_chain = ORobotDefault::from_urdf("b1");
    let z1_chain = ORobotDefault::from_urdf("z1");

    let mut robot_set = ORobotSetDefault::new_empty();
    robot_set.add_robot(b1_chain, 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Floating, OJointLimit::new_manual(vec![0.0; 6], vec![-1.0; 6], vec![1.0; 6], vec![1.0; 6]));
    robot_set.add_robot(z1_chain, 1, 0, &Isometry3::from_constructors(&[0.1, 0.0, 0.02], &[0.0; 3]), [0.0; 3], OJointType::Fixed, OJointLimit::default());

    robot_set.as_robot().bevy_display();
}