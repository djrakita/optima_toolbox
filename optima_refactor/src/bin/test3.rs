use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::{ORobotDefault, SaveRobot};

fn main() {
    let mut robot = ORobotDefault::load_from_saved_robot("ur5");
    robot.bevy_self_collision_visualization();
}