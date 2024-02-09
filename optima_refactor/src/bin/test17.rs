use optima_bevy::optima_bevy_utils::robotics::{BevyRoboticsTraitF64};
use optima_robotics::robot::ORobotDefault;

fn main() {
    let robot = ORobotDefault::load_from_saved_robot("ur5");
    robot.bevy_ik_test(9, &[1.0; 6]);
}