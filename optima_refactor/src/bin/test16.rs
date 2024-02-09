use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::{ORobotDefault};

fn main() {
    let mut r = ORobotDefault::load_from_saved_robot("ur5");
    r.bevy_self_collision_visualization();
    // r.preprocess(SaveRobot::Save(Some("ryan_test")));

    // r.set_dead_end_link(4);
    r.set_joint_as_fixed(5, &[0.0]);
    // r.save_robot(Some("ryan_test"));
    // r.bevy_display();
    // r.bevy_display();
}