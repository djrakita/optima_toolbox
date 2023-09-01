use bevy::app::App;
use bevy::DefaultPlugins;
use bevy::prelude::{ClearColor, Color, Msaa, PluginGroup, Window};
use bevy::window::WindowPlugin;

pub fn bevy_base(app: &mut App) {
    app
        .insert_resource(ClearColor(Color::rgb(1.0, 0.,0.)))
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