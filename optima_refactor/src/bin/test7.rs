use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::{ORobotDefault, SaveRobot};

fn main() {
    let mut r = ORobotDefault::load_from_saved_robot("xarm7_with_gripper_and_rail");
    r.preprocess(SaveRobot::Save(None));
    r.bevy_self_collision_visualization();
}