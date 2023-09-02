use ad_trait::AD;
use bevy::prelude::*;
use bevy_stl::StlPlugin;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::OLinalgTrait;
use optima_robotics::robot::ORobot;
use crate::optima_bevy_utils::camera::CameraSystems;
use crate::optima_bevy_utils::lights::LightSystems;
use crate::optima_bevy_utils::robotics::{BevyORobot, RoboticsSystems};

pub mod scripts;
pub mod optima_bevy_utils;


pub trait OptimaBevyTrait {
    fn optima_bevy_base(&mut self) -> &mut Self;
    fn optima_bevy_pan_orbit_camera(&mut self) -> &mut Self;
    fn optima_bevy_starter_lights(&mut self) -> &mut Self;
    fn optima_bevy_spawn_robot<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static>(&mut self, robot: ORobot<T, P, L>) -> &mut Self;
}
impl OptimaBevyTrait for App {
    fn optima_bevy_base(&mut self) -> &mut Self {
        self
            .insert_resource(ClearColor(Color::rgb(0.8, 0.8, 0.8)))
            .insert_resource(Msaa::Sample4)
            .add_plugins(DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "OPTIMA".to_string(),
                        ..Default::default()
                    }),
                    ..Default::default()
                })
            )
            .add_plugins(StlPlugin);
        self
    }

    fn optima_bevy_pan_orbit_camera(&mut self) -> &mut Self {
        self
            .add_systems(Startup, CameraSystems::system_spawn_pan_orbit_camera)
            .add_systems(Update, CameraSystems::system_pan_orbit_camera);

        self
    }

    fn optima_bevy_starter_lights(&mut self) -> &mut Self {
        self
            .add_systems(Startup, LightSystems::starter_point_lights);

        self
    }

    fn optima_bevy_spawn_robot<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static>(&mut self, robot: ORobot<T, P, L>) -> &mut Self {
        self.insert_resource(BevyORobot(robot));
        self.add_systems(Startup, RoboticsSystems::system_spawn_robot_links_as_stl_meshes::<T, P, L>);

        self
    }
}