use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::{ORobotDefault, SaveRobot};

fn main() {
    let mut r = ORobotDefault::load_from_saved_robot("panda7_2");
    // r.set_joint_as_fixed(9, &[0.025]);
    // r.preprocess(SaveRobot::Save(Some("panda7_2")), None, None);
    // r.parry_shape_scene_compute_average_distances(SaveRobot::Save(None), Some(2000));
    r.bevy_self_collision_visualization();
    // r.bevy_display();
}
