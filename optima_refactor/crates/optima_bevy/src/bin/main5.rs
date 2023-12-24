use bevy::app::App;
use bevy::prelude::Color;
use optima_bevy::OptimaBevyTrait;
use optima_interpolation::splines::{BSpline, InterpolatingSpline, InterpolatingSplineType, SplineConstructorLinear, SplineConstructorTrait};

fn main() {
    let s = vec![ [0.,0.,0.], [1.,1.,1.], [0.5, 0.2, 0.9], [0.2, 0.1, 0.1], [0.4, 0.5, 0.6], [0.1, 0.7, 0.3] ];
    // let spline = InterpolatingSpline::new(s, InterpolatingSplineType::HermiteCubic);
    let spline = BSpline::new(s, 5);
    // let spline = SplineConstructorLinear.construct(s);

    let mut app = App::new();
    app.optima_bevy_starter_scene();
    app.optima_bevy_draw_3d_curve(spline, 100, 10., 5, 2);
    app.run();
}