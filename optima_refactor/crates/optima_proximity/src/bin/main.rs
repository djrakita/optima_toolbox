extern crate parry_ad;

use parry_ad::na::{Isometry3, Vector3};
use parry_ad::shape::{Ball, Cuboid};
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_proximity::distance::ODistanceTrait;
use optima_proximity::ParryShapeWrapper;

fn main() {
    let a = Cuboid::<f64>::new(Vector3::new(1.,1.,1.));
    let b = Ball::new(1.0);

    let wa = ParryShapeWrapper::new(a);
    let wb = ParryShapeWrapper::new(b);

    let res = wa.distance(&Isometry3::identity(), &wb, &Isometry3::from_constructors(&[4.,0.,0.], &[0.0; 3]));
    println!("{:?}", res);
}