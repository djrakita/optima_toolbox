use optima_interpolation::linearly_interpolate_points;
use optima_interpolation::splines::SplineConstructor;
use optima_linalg::VecOfOVecTrait;

fn main() {
    let sc = SplineConstructor::Linear;
    let init = sc.get_default_initial_condition([0.,0.,0.], [1.,2.,3.], 15);
    println!("{:?}", init);
    let res = init.concatenate_all();
    println!("{:?}", res);
}