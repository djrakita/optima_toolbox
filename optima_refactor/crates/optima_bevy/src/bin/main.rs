use bevy::prelude::*;
use optima_bevy::scripts::{bevy_base, bevy_pan_orbit_camera, spawn_cube};

fn main() {
    let mut app = App::new();
    bevy_base(&mut app);
    bevy_pan_orbit_camera(&mut app);
    spawn_cube(&mut app);
    app.run();
}