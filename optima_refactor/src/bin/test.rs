use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};

fn main() {
    let s = InterpolatingSpline::new(vec![vec![0.0], vec![1.0], vec![2.0], vec![3.0], vec![4.0], vec![5.0], vec![6.0], vec![7.0], vec![8.0]], InterpolatingSplineType::BezierCubic);

    let res = s.interpolating_spline_interpolate(0.0);
    println!("{:?}", res);
}