use parry_ad::na::Point3;
use parry_ad::shape::ConvexPolyhedron;
use optima_3d_spatial::optima_3d_vec::O3DVec;

fn main() {
    let a = ConvexPolyhedron::from_convex_hull(&[Point3::new(2.,0.,0.), Point3::new(1.,1.,0.), Point3::new(1.,0.,0.)]);
    let a = a.unwrap();
    let aabb = a.local_aabb();
    let bs = a.local_bounding_sphere();
    println!("{:?}", aabb);

    let center = 0.5*(aabb.mins.add(&aabb.maxs));
    println!("{:?}", center);
    println!("{:?}", aabb.maxs.x - aabb.mins.x);
    println!("{:?}", aabb.maxs.y - aabb.mins.y);
    println!("{:?}", aabb.maxs.z - aabb.mins.z);
}