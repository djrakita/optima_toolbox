use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_interpolation::{InterpolatorTrait};
use optima_robotics::robot::{ORobotDefault, SaveRobot};

fn main() {
    let waypoints = vec![ vec![0.0; 7], vec![1.0; 7], vec![1.1; 7] ];

    let s = InterpolatingSpline::new(waypoints, InterpolatingSplineType::Linear)
        .to_arclength_parameterized_interpolator(100)
        .to_timed_interpolator(5.0);

    // let r = ORobotDefault::from_urdf("ur5");

    // r.bevy_motion_playback(&s);

    let mut r = ORobotDefault::load_from_saved_robot("panda7");
    // r.bevy_display();
    r.bevy_motion_playback(&s);
}