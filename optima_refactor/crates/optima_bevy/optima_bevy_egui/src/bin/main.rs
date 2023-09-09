use std::sync::Mutex;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::{EguiContexts, EguiPlugin};
use bevy_egui::egui::{Color32, Pos2, Rounding, Visuals};
use bevy_egui::egui::panel::{Side, TopBottomSide};
use optima_bevy_egui::{OEguiButton, OEguiContainerTrait, OEguiEngine, OEguiEngineWrapper, OEguiSelector, OEguiSelectorMode, OEguiSidePanel, OEguiSlider, OEguiTopBottomPanel, OEguiWindow};
use optima_bevy_egui::OEguiWidgetTrait;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(LogDiagnosticsPlugin::default())
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(EguiPlugin)
        .insert_resource(OEguiEngineWrapper { 0: Mutex::new(OEguiEngine::new()) })
        // Systems that create Egui widgets should be run during the `CoreSet::Update` set,
        // or after the `EguiSet::BeginFrame` system (which belongs to the `CoreSet::PreUpdate` set).
        .add_systems(Update, ui_example_system)
        .add_systems(Last, ui_example_system2.after(ui_example_system))
        .add_systems(Last, ui_example_system3.after(ui_example_system2))
        .run();
}

fn ui_example_system(mut contexts: EguiContexts, egui_engine: Res<OEguiEngineWrapper>, keys: Res<Input<KeyCode>>, window_query: Query<&Window, With<PrimaryWindow>>) {
    if keys.just_pressed(KeyCode::A) {
        egui_engine.get_mutex_guard().open_window("window");
        egui_engine.get_mutex_guard().set_position_of_window("window", Pos2::new(1000.,500.));
    }

    let mut visuals = Visuals::dark();
    visuals.widgets.noninteractive.bg_fill = Color32::from_rgba_premultiplied(150, 20, 20, 10);
    contexts.ctx_mut().set_visuals(visuals);

    if keys.pressed(KeyCode::Space) {
        egui_engine.get_mutex_guard().set_position_of_window("window", Pos2::new(500.,500.));
    }

    OEguiWindow::new("test", true, true, true, true, true, true)
        .show("window", contexts.ctx_mut(), &egui_engine, &window_query, &(), |ui| {
            OEguiButton::new("button")
                .show("button1", ui, &egui_engine, &());

            OEguiSlider::new(-1.0, 1.0)
                .show("slider1", ui, &egui_engine, &());

            OEguiSelector::new(OEguiSelectorMode::Checkboxes, vec!["1", "2", "3"], None, true)
                .show("selector1", ui, &egui_engine, & *keys);
        });

    OEguiSidePanel::new(Side::Left, 300.0)
        .show("side_panel", contexts.ctx_mut(), &egui_engine, &window_query, &(), |ui| {
            OEguiButton::new("hello")
                .show("button3", ui, &egui_engine, &());
        });


    OEguiTopBottomPanel::new(TopBottomSide::Top, 200.0)
        .show("top_panel", contexts.ctx_mut(), &egui_engine, &window_query, &(), |ui| {
            OEguiButton::new("hello")
                .show("button4", ui, &egui_engine, &());
        });

    if keys.pressed(KeyCode::B) {
        egui_engine.get_mutex_guard().close_side_panel("side_panel");
    }
    if keys.pressed(KeyCode::C) {
        egui_engine.get_mutex_guard().open_side_panel("side_panel");
    }
    if keys.pressed(KeyCode::D) {
        egui_engine.get_mutex_guard().open_top_bottom_panel("top_panel");
    }
    if keys.pressed(KeyCode::E) {
        egui_engine.get_mutex_guard().close_top_bottom_panel("top_panel");
    }
}

fn ui_example_system2(egui_engine: Res<OEguiEngineWrapper>) {
    let egui_engine = egui_engine.get_mutex_guard();

    let a = egui_engine.ui_contains_pointer();
    if a { } // println!("yes"); }
}

fn ui_example_system3(egui_engine: Res<OEguiEngineWrapper>) {
    let mut engine = egui_engine.get_mutex_guard();
    engine.reset_on_frame();
}