use bevy::app::{App};
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::egui::panel::Side;
use bevy_egui::EguiContexts;
use optima_bevy::OptimaBevyTrait;
use optima_bevy_egui::{OEguiButton, OEguiContainerTrait, OEguiEngineWrapper, OEguiSidePanel, OEguiTextbox, OEguiWidgetTrait};

/*
#[derive(Default)]
pub struct TestNode;
impl bevy::render::render_graph::Node for TestNode {
    fn run(&self, graph: &mut RenderGraphContext, _render_context: &mut RenderContext, _world: &World) -> Result<(), NodeRunError> {

        Ok(())
    }
}
*/

fn main() {
    let mut app = App::new();
    app
        .optima_bevy_base()
        .optima_bevy_robotics_scene_visuals_starter()
        .optima_bevy_pan_orbit_camera()
        .optima_bevy_starter_lights()
        .optima_bevy_egui();

    app.add_systems(Update, test_system);
    app.run();
}

fn test_system(mut contexts: EguiContexts, egui_engine: Res<OEguiEngineWrapper>, _keys: Res<Input<KeyCode>>, window_query: Query<&Window, With<PrimaryWindow>>) {
    OEguiSidePanel::new(Side::Left, 100.0)
        .show("test", contexts.ctx_mut(), &egui_engine, &window_query, &(), |ui| {
            OEguiButton::new("tester")
                .show("tester", ui, &egui_engine, &());

            OEguiTextbox::new(false)
                .show("text", ui, &egui_engine, &());
        });
}
