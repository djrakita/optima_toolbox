use ad_trait::AD;
use bevy::prelude::*;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::OLinalgTrait;
use optima_robotics::robot::ORobot;
use crate::optima_bevy_utils::robotics::{UpdaterRobotState};
use crate::OptimaBevyTrait;

pub fn test_script<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static>(robot: ORobot<T, P, L>) {
    App::new()
        .optima_bevy_base()
        .optima_bevy_robotics_base(robot)
        .optima_bevy_starter_lights()
        .optima_bevy_pan_orbit_camera()
        .optima_bevy_spawn_robot::<T, P, L>()
        .optima_bevy_robotics_scene_visuals_starter()
        .add_systems(Update, tt::<T, P, L>)
        .run()
}

fn tt<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static>(keys: Res<Input<KeyCode>>,
                                                                 mut updater: ResMut<UpdaterRobotState>) {
    if keys.just_pressed(KeyCode::Space) {
        updater.add_update_request(0, &[0.2; 18])
    }
}
