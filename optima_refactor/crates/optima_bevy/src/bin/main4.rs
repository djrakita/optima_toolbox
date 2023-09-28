use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::{O3DPose};
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::chain::{OChainDefault};
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};

fn main() {
    let b1_chain = OChainDefault::from_urdf("b1");
    let z1_chain = OChainDefault::from_urdf("z1");

    let mut robot = ORobotDefault::new_empty();
    robot.add_chain(b1_chain, 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Floating, OJointLimit::new_manual(vec![0.0; 6], vec![-1.0; 6], vec![1.0; 6], vec![1.0; 6]));
    robot.add_chain(z1_chain, 1, 0, &Isometry3::from_constructors(&[0.1, 0.0, 0.02], &[0.0; 3]), [0.0; 3], OJointType::Fixed, OJointLimit::default());

    robot.bevy_display();
}