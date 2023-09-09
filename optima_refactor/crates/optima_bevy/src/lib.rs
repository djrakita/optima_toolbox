use ad_trait::AD;
use bevy::input::common_conditions::input_just_pressed;
use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use bevy_mod_picking::debug::{DebugPickingMode};
use bevy_mod_picking::DefaultPickingPlugins;
use bevy_stl::StlPlugin;
use bevy_transform_gizmo::TransformGizmoPlugin;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_bevy_egui::{OEguiEngineWrapper};
use optima_linalg::OLinalgTrait;
use optima_robotics::robot::ORobot;
use crate::optima_bevy_utils::camera::CameraSystems;
use crate::optima_bevy_utils::lights::LightSystems;
use crate::optima_bevy_utils::robotics::{BevyORobot, RoboticsSystems, UpdaterRobotState};
use crate::optima_bevy_utils::viewport_visuals::ViewportVisualsSystems;

pub mod scripts;
pub mod optima_bevy_utils;

pub trait OptimaBevyTrait {
    fn optima_bevy_base(&mut self) -> &mut Self;
    fn optima_bevy_robotics_base<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static>(&mut self, robot: ORobot<T, P, L>) -> &mut Self;
    fn optima_bevy_pan_orbit_camera(&mut self) -> &mut Self;
    fn optima_bevy_starter_lights(&mut self) -> &mut Self;
    fn optima_bevy_spawn_robot<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static>(&mut self) -> &mut Self;
    fn optima_bevy_robotics_scene_visuals_starter(&mut self) -> &mut Self;
    fn optima_bevy_egui(&mut self) -> &mut Self;
}
impl OptimaBevyTrait for App {
    fn optima_bevy_base(&mut self) -> &mut Self {
        self
            .insert_resource(ClearColor(Color::rgb(0.5, 0.5, 0.5)))
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
            .add_plugins( DefaultPickingPlugins)
            .add_systems(
                Update,
                (
                    (|mut next: ResMut<NextState<_>>| next.set(DebugPickingMode::Normal)).run_if(in_state(DebugPickingMode::Disabled)),
                    (|mut next: ResMut<NextState<_>>| next.set(DebugPickingMode::Disabled)).run_if(in_state(DebugPickingMode::Normal)),
                )
                    .distributive_run_if(input_just_pressed(KeyCode::F3)),
            )
            .add_systems(
                Startup,
                |mut next: ResMut<NextState<_>>| next.set(DebugPickingMode::Disabled)
            )
            .add_plugins(TransformGizmoPlugin::default())
            .add_plugins(StlPlugin);

        self
    }
    fn optima_bevy_robotics_base<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static>(&mut self, robot: ORobot<T, P, L>) -> &mut Self {
        self
            .insert_resource(BevyORobot(robot))
            .insert_resource(UpdaterRobotState::new())
            .add_systems(Last, RoboticsSystems::system_robot_state_updater::<T, P, L>);

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
    fn optima_bevy_spawn_robot<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static>(&mut self) -> &mut Self {
        self.add_systems(Startup, RoboticsSystems::system_spawn_robot_links_as_stl_meshes::<T, P, L>);

        self
    }
    fn optima_bevy_robotics_scene_visuals_starter(&mut self) -> &mut Self {
        self
            .add_systems(Startup, ViewportVisualsSystems::system_draw_robotics_grid);

        self
    }
    fn optima_bevy_egui(&mut self) -> &mut Self {
        self
            .add_plugins(EguiPlugin)
            .insert_resource(OEguiEngineWrapper::new());

        self
    }
}