use parry3d_f64::na::Vector3;
use parry3d_f64::query::PointQuery;
use parry3d_f64::shape::{Ball, Cuboid};
use parry_ad::na::Point3;

fn main() {
    let c = Cuboid::new(Vector3::new(1.,1.,1.));

    let res = c.project_local_point(&Point3::new(1.1,0.,0.), false);
    println!("{:?}", res);

    let s = Ball::new(2.0);
    let res = s.project_local_point(&Point3::new(1.1,0.2,0.), false);
    println!("{:?}", res.is_inside);
}