use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::{ORobotDefault};
use optima_robotics::robot::SaveRobot::Save;

fn main() {
    let mut robot = ORobotDefault::from_urdf("panda");
    robot.preprocess(Save(None));
    robot.bevy_self_collision_visualization();
}