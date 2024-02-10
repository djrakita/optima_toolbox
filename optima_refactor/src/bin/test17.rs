
use optima_bevy::optima_bevy_utils::robotics::{BevyRoboticsTrait, BevyRoboticsTraitF64};
use optima_robotics::robot::ORobotDefault;

fn main() {
    let robot = ORobotDefault::load_from_saved_robot("xarm7_with_gripper_and_rail");
    // robot.bevy_display();
    robot.bevy_ik_test(19, &[0.1; 9]);
    // let q = UnitQuaternion::from_scaled_axis_of_rotation(&[1.1,2.1,3.1]);
    // let q2 = UnitQuaternion::from_scaled_axis_of_rotation(&[1.,2.,3.]);
    // println!("{:?}", q.dis(&q2));

    // println!("{:?}", q.ln().norm());
    // println!("{:?}", q.scaled_axis().norm());
}