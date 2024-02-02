use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::ORobotDefault;

fn main() {
    let r = ORobotDefault::load_from_saved_robot("xarm7_bimanual_viewpoint");
    r.bevy_display();
}