use ad_trait::AD;
use bevy::input::common_conditions::input_just_pressed;
use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use bevy_mod_picking::debug::{DebugPickingMode};
use bevy_mod_picking::DefaultPickingPlugins;
use bevy_prototype_debug_lines::{DebugLinesPlugin};
use bevy_stl::StlPlugin;
use bevy_transform_gizmo::TransformGizmoPlugin;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategory};
use optima_bevy_egui::{OEguiEngineWrapper};
use optima_interpolation::{InterpolatorTrait};
use optima_linalg::{OLinalgCategory, OVec};
use optima_proximity::shape_scene::{OParryGenericShapeScene, ShapeSceneTrait};
use optima_proximity::shapes::OParryShape;
use optima_robotics::robot::ORobot;
use optima_robotics::robotics_traits::AsRobotTrait;
use optima_universal_hashmap::AnyHashmap;
use crate::optima_bevy_utils::camera::CameraSystems;
use crate::optima_bevy_utils::lights::LightSystems;
use crate::optima_bevy_utils::robotics::{BevyORobot, RoboticsSystems, RobotStateEngine};
use crate::optima_bevy_utils::shape_scene::{ShapeSceneActions, ShapeSceneType};
use crate::optima_bevy_utils::storage::BevyAnyHashmap;
use crate::optima_bevy_utils::transform::TransformUtils;
use crate::optima_bevy_utils::viewport_visuals::{BevyDrawShape, ViewportVisualsActions, ViewportVisualsSystems};

pub mod scripts;
pub mod optima_bevy_utils;

pub trait OptimaBevyTrait {
    fn optima_bevy_starter_scene(&mut self) -> &mut Self;
    fn optima_bevy_base(&mut self) -> &mut Self;
    fn optima_bevy_robotics_base<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static, A: AsRobotTrait<T, C, L>>(&mut self, as_chain: A) -> &mut Self;
    fn optima_bevy_pan_orbit_camera(&mut self) -> &mut Self;
    fn optima_bevy_starter_lights(&mut self) -> &mut Self;
    fn optima_bevy_spawn_robot<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static>(&mut self) -> &mut Self;
    fn optima_bevy_robotics_scene_visuals_starter(&mut self) -> &mut Self;
    fn optima_bevy_egui(&mut self) -> &mut Self;
    fn optima_bevy_draw_3d_curve<T: AD, V: OVec<T>, I: InterpolatorTrait<T, V> + 'static + Sync + Send>(&mut self, curve: I, num_points: usize, width_in_mm: f32, num_points_per_circle: usize, num_concentric_circles: usize) -> &mut Self;
    fn optima_bevy_draw_shape<T: AD, P: O3DPose<T>>(&mut self, shape: BevyDrawShape<T>, pose: P) -> &mut Self;
    fn optima_bevy_spawn_robot_shape_scene<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static, V: OVec<T>>(&mut self, robot: ORobot<T, C, L>, state: V) -> &mut Self;
    fn optima_bevy_spawn_generic_shape_scene<T: AD, P: O3DPose<T>>(&mut self, scene: OParryGenericShapeScene<T, P>) -> &mut Self;
}
impl OptimaBevyTrait for App {
    fn optima_bevy_starter_scene(&mut self) -> &mut Self {
        self
            .optima_bevy_base()
            .optima_bevy_pan_orbit_camera()
            .optima_bevy_egui()
            .optima_bevy_starter_lights()
            .optima_bevy_robotics_scene_visuals_starter();

        self
    }
    fn optima_bevy_base(&mut self) -> &mut Self {
        self
            .insert_resource(ClearColor(Color::rgb(0.5, 0.5, 0.5)))
            .insert_resource(Msaa::default())
            .insert_resource(BevyAnyHashmap(AnyHashmap::new()))
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
            .add_plugins(StlPlugin)
            .add_plugins(DebugLinesPlugin::default());

        self
    }
    fn optima_bevy_robotics_base<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static, A: AsRobotTrait<T, C, L>>(&mut self, as_robot: A) -> &mut Self {
        self
            .insert_resource(BevyORobot(as_robot.as_robot().clone(), 0))
            .insert_resource(RobotStateEngine::new())
            .add_systems(Last, RoboticsSystems::system_robot_state_updater::<T, C, L>);

        self
    }
    fn optima_bevy_pan_orbit_camera(&mut self) -> &mut Self {
        self
            .add_systems(Startup, CameraSystems::system_spawn_pan_orbit_camera)
            .add_systems(PostUpdate, CameraSystems::system_pan_orbit_camera.in_set(BevySystemSet::Camera));

        self
    }
    fn optima_bevy_starter_lights(&mut self) -> &mut Self {
        self
            .add_systems(Startup, LightSystems::starter_point_lights);

        self
    }
    fn optima_bevy_spawn_robot<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static>(&mut self) -> &mut Self {
        self.add_systems(Startup, RoboticsSystems::system_spawn_robot_links_as_stl_meshes::<T, C, L>);

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
            .insert_resource(OEguiEngineWrapper::new())
            .add_systems(Last, |egui_engine: Res<OEguiEngineWrapper>| { egui_engine.get_mutex_guard().reset_on_frame() });

        self
    }
    fn optima_bevy_draw_3d_curve<T: AD, V: OVec<T>, I: InterpolatorTrait<T, V> + 'static + Sync + Send>(&mut self, curve: I, num_points: usize, width_in_mm: f32, num_points_per_circle: usize, num_concentric_circles: usize) -> &mut Self {
        // mut lines: ResMut<DebugLines>
        self.add_systems(Update, move |mut gizmos: Gizmos| {
            let mut curr_val = 0.0;
            let stride_length = 1.0 / (num_points as f64);
            let mut t_vals = vec![];
            while curr_val <= 1.0 { t_vals.push(curr_val); curr_val += stride_length; }
            for i in 0..t_vals.len() - 1 {
                let curr_t = t_vals[i];
                let next_t = t_vals[i+1];
                let curr_point = curve.interpolate_normalized(T::constant(curr_t));
                let next_point = curve.interpolate_normalized(T::constant(next_t));
                assert_eq!(curr_point.len(), 3);

                let curr_point = TransformUtils::util_convert_z_up_ovec_to_z_up_vec3(&curr_point);
                let next_point = TransformUtils::util_convert_z_up_ovec_to_z_up_vec3(&next_point);

                // ViewportVisualsActions::action_draw_gpu_line_optima_space(&mut lines, curr_point, next_point, Color::rgb(0.,0.5, 1.0), width_in_mm, num_points_per_circle, num_concentric_circles, 0.0);

                ViewportVisualsActions::action_draw_gpu_line_optima_space_gizmo(&mut gizmos, curr_point, next_point, Color::rgb(0.0, 0.5, 1.0), width_in_mm, num_points_per_circle, num_concentric_circles);

                curr_val += stride_length;
            }
        });

        self
    }
    fn optima_bevy_draw_shape<T: AD, P: O3DPose<T>>(&mut self, shape: BevyDrawShape<T>, pose: P) -> &mut Self {
        self.add_systems(Startup, move |mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>| {
            ViewportVisualsActions::action_draw_shape(&shape, &pose, &mut commands, &mut meshes, &mut materials);
        });

        self
    }
    fn optima_bevy_spawn_robot_shape_scene<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static, V: OVec<T>>(&mut self, robot: ORobot<T, C, L>, state: V) -> &mut Self {
        self.add_systems(Startup, move |mut commands: Commands, asset_server: Res<AssetServer>, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>| {
            ShapeSceneActions::action_spawn_shape_scene(&robot, state.ovec_as_slice(), ShapeSceneType::Robot, &mut commands, &asset_server, &mut meshes, &mut materials);
        });

        self
    }

    fn optima_bevy_spawn_generic_shape_scene<T: AD, P: O3DPose<T>>(&mut self, scene: OParryGenericShapeScene<T, P>) -> &mut Self {
        self.add_systems(Startup, move |mut commands: Commands, asset_server: Res<AssetServer>, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>| {
            ShapeSceneActions::action_spawn_shape_scene(&scene, (), ShapeSceneType::Environment, &mut commands, &asset_server, &mut meshes, &mut materials);
        });

        self
    }

}

#[derive(Clone, Debug, SystemSet, Hash, PartialEq, Eq)]
pub enum BevySystemSet {
    Camera,
    GUI
}
