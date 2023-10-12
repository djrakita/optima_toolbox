use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::ORobotDefault;

fn main() {
    let chain = ORobotDefault::from_urdf("b1");
    chain.bevy_display();
}