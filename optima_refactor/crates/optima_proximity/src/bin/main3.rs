use parry_ad::na::{Isometry3, Vector3};
use optima_3d_spatial::optima_3d_pose::O3DPose;

fn main() {
    let a = parry_ad::shape::Cuboid::new(Vector3::new(1.,1.,1.));
    let b = parry_ad::shape::Cuboid::new(Vector3::new(1.,1.,1.));
    let pa = Isometry3::from_constructors(&[0.,0.,0.], &[0.,0.,0.]);
    let pb = Isometry3::from_constructors(&[3.,0.,0.], &[0.,0.,0.]);
    let res = parry_ad::query::distance(&pa, &a, &pb, &b);
    println!("{:?}", res);

    let a = parry3d_f64::shape::Cuboid::new(Vector3::new(1.,1.,1.));
    let b = parry3d_f64::shape::Cuboid::new(Vector3::new(1.,1.,1.));
    let pa = Isometry3::from_constructors(&[0.,0.,0.], &[0.,0.,0.]);
    let pb = Isometry3::from_constructors(&[3.,0.,0.], &[0.,0.,0.]);
    let res = parry3d_f64::query::distance(&pa, &a, &pb, &b);
    println!("{:?}", res);
}