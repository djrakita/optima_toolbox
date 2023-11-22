use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::{ORobotDefault};

fn main() {
    let mut r = ORobotDefault::load_from_saved_robot("z1");
    // r.parry_shape_scene_compute_average_distances(SaveRobot::Save(None), Some(2000));
    r.bevy_self_collision_visualization();
}
