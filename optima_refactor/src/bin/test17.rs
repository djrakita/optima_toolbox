
use optima_bevy::optima_bevy_utils::robotics::{BevyRoboticsTrait, BevyRoboticsTraitF64};
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_optimization::robotics_optimization_functions::{AxisDirection, LookAtTarget};

fn main() {
    let robot = ORobotDefault::load_from_saved_robot("xarm7_bimanual_viewpoint");
    // robot.bevy_display();
    robot.bevy_ik_lookat(20, &vec![0.0,0.0,-1.5,0.0,0.3,0.0,0.24,0.0,0.2,3.7,-1.44,0.0,0.3,0.0,0.2,0.0], 32, AxisDirection::Z, AxisDirection::Y, LookAtTarget::RobotLink(20));

    // let q = UnitQuaternion::from_scaled_axis_of_rotation(&[1.1,2.1,3.1]);
    // let q2 = UnitQuaternion::from_scaled_axis_of_rotation(&[1.,2.,3.]);
    // println!("{:?}", q.dis(&q2));

    // println!("{:?}", q.ln().norm());
    // println!("{:?}", q.scaled_axis().norm());
}