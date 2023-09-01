use bevy::prelude::*;
use optima_bevy::scripts::bevy_base;

fn main() {
    let mut app = App::new();
    bevy_base(&mut app);
    app.run();
}