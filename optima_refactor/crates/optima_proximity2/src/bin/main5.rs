use std::time::Instant;
use parry_ad::na::Isometry3;
use parry_ad::shape::Ball;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use optima_proximity2::global_shapes::GlobalShapes;
use optima_proximity2::shapes::OParryShape;

fn main() {
    let shape = OParryShape::new(Ball::new(2.0), Isometry3::identity());

    GlobalShapes::register_shape(100000, shape);

    let start = Instant::now();
    for _ in 0..1000 {
        let s = GlobalShapes::get_shape::<OParryShape<f64, Isometry3<f64>>>(100000);
    }
    println!("{:?}", start.elapsed());

    let v: Vec<usize> = (0..12).collect();
    v.par_iter().for_each(|x| {
        let s = GlobalShapes::get_shape::<OParryShape<f64, Isometry3<f64>>>(100000).unwrap();
        println!("{:?}", s.base_shape().obb_max_dis_error());
        GlobalShapes::register_shape(12, OParryShape::new(Ball::new(2.0), Isometry3::identity()));
    });
}