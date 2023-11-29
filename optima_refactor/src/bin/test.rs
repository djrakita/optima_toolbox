use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::{ORobotDefault, SaveRobot};
use optima_robotics::robot_set::ORobotSetDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};
use optima_robotics::robotics_traits::AsRobotTrait;

fn main() {
    // let r1 = ORobotDefault::load_from_saved_robot("xarm7_with_gripper_and_rail");
    // let r2 = r1.clone();
    //
    // let mut set = ORobotSetDefault::new_empty();
    // set.add_robot(r1, 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    // set.add_robot(r2, 0, 0, &Isometry3::from_constructors(&[0.9, 0.,0.], &[0.0; 3]), [0.0; 3], OJointType::Fixed, OJointLimit::default());

    let mut r = ORobotDefault::load_from_saved_robot("xarm7_bimanual_viewpoint");
    // r.set_dead_end_link(33);
    // r.save_robot(Some("xarm7_bimanual_viewpoint"));
    // r.set_joint_as_fixed(9, &[0.025]);
    // r.parry_shape_scene_compute_average_distances(SaveRobot::Save(Some("panda7_2")), Some(10_000));
    // r.preprocess(SaveRobot::Save(Some("xarm7_bimanual")), None, Some(10_000));
    // r.parry_shape_scene_compute_average_distances(SaveRobot::Save(None), Some(2000));
    r.bevy_self_collision_visualization();
    // r.bevy_display();
}
