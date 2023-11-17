use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_interpolation::{InterpolatorTrait, TimedInterpolator};
use optima_robotics::robot::ORobotDefault;

fn main() {
    let waypoints = vec![ vec![0.0; 8], vec![1.0; 8], vec![1.2; 8] ];
    let s = InterpolatingSpline::new(waypoints, InterpolatingSplineType::Linear)
        .to_arclength_parameterized_interpolator(100)
        .to_timed_interpolator(3.0);

    let r = ORobotDefault::from_urdf("panda");

    r.bevy_motion_playback(&s);
}