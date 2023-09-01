use bevy::app::App;
use bevy::DefaultPlugins;
use bevy::pbr::PbrBundle;
use bevy::prelude::{Assets, ClearColor, Color, Commands, Mesh, Msaa, PluginGroup, ResMut, shape, StandardMaterial, Startup, Update, Window};
use bevy::window::WindowPlugin;
use crate::optima_bevy_utils::camera::CameraSystems;

pub fn bevy_base(app: &mut App) {
    app
        .insert_resource(ClearColor(Color::rgb(0.8, 0.8,0.8)))
        .insert_resource(Msaa::Sample4)
        .add_plugins(DefaultPlugins
            .set(WindowPlugin {
                primary_window: Some(Window {
                    title: "OPTIMA".to_string(),
                    ..Default::default()
                }),
                ..Default::default()
            })
        );
}

pub fn bevy_pan_orbit_camera(app: &mut App) {
    app
        .add_systems(Startup, CameraSystems::system_spawn_pan_orbit_camera)
        .add_systems(Update, CameraSystems::system_pan_orbit_camera);
}

pub fn spawn_cube(app: &mut App) {
    app.add_systems(Update, spawn_cube_sys);
}

fn spawn_cube_sys(mut commands: Commands,
                  mut meshes: ResMut<Assets<Mesh>>,
                  mut materials: ResMut<Assets<StandardMaterial>>) {
    commands.spawn(
        PbrBundle {
            mesh: meshes.add(shape::Cube::default().into()),
            material: materials.add(StandardMaterial::default()),
            transform: Default::default(),
            ..Default::default()
        }
    );
}
