use ad_trait::forward_ad::adf::adf_f32x2;
use bevy::prelude::App;
use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion, O3DPose};
use optima_bevy::scripts::{bevy_base, bevy_pan_orbit_camera, spawn_cube, spawn_robot};
use optima_linalg::NalgebraLinalg;
use optima_robotics::chain::OChain;
use optima_robotics::robot::ORobot;
use optima_robotics::robotics_components::{OJointLimit, OJointType};

fn main() {
    let robot = ORobot::<f64, ImplicitDualQuaternion<_>, NalgebraLinalg>::new_from_single_chain_name("z1");

    let mut app = App::new();
    bevy_base(&mut app);
    bevy_pan_orbit_camera(&mut app);
    spawn_robot(&mut app, robot);
    // spawn_cube(&mut app);
    app.run();
}