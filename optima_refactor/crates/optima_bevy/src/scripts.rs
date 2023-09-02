use ad_trait::AD;
use bevy::prelude::App;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::OLinalgTrait;
use optima_robotics::robot::ORobot;
use crate::OptimaBevyTrait;

pub fn test_script<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static>(robot: ORobot<T, P, L>) {
    App::new()
        .optima_bevy_base()
        .optima_bevy_starter_lights()
        .optima_bevy_pan_orbit_camera()
        .optima_bevy_spawn_robot(robot)
        .run()
}
