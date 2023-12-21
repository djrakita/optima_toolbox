use std::sync::{Arc, Mutex};
use bevy::app::App;
use bevy::DefaultPlugins;
use bevy::prelude::{Res, Time, Update};
use optima_robotics::robot::{ORobotDefault};

fn test(robot: &Arc<ORobotDefault>, time: &Res<Time>, a: &Arc<Mutex<i32>>) {
    let mut binding = a.lock().unwrap();
    *binding += 1;
    println!("{:?}, {:?}, {:?}", robot.num_dofs(), time.elapsed(), binding);
}

fn test2(robot: &Arc<ORobotDefault>, a: &Arc<Mutex<i32>>) {
    println!("{:?}, {:?}", robot.get_dof_upper_bounds(), a);
}

fn main() {
    let r = ORobotDefault::load_from_saved_robot("ur5");
    let r1 = Arc::new(r);
    let r2 = r1.clone();
    let a = Arc::new(Mutex::new(1));
    let b = a.clone();

    let mut app = App::new();
    app.add_plugins(DefaultPlugins);
    app.add_systems(Update, move |time: Res<Time>| {
        test(&r1, &time, &a);
    });
    app.add_systems(Update, move || {
        test2(&r2, &b);
    });
    app.run();
}