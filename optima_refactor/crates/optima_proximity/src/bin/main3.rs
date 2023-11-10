use parry_ad::na::{Isometry3, Vector3};
use parry_ad::shape::{Ball, Cuboid};
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_proximity2::pair_queries::{OPairQryTrait, ParryContactQry, ParryDistanceQry, ParryIntersectQry};
use optima_proximity2::shape_queries::{OShpQryContactTrait, OShpQryIntersectTrait};
use optima_proximity2::shapes::{OParryShape, ParryShapeRep, ParryQryShapeType, ParryDistanceOutput, ParryDisMode, OParryBoundingSphere};

fn main() {
    let s1 = OParryShape::new_with_path_option(Cuboid::new(Vector3::new(1., 1., 1.)), Isometry3::identity());
    let s2 = OParryShape::new_with_path_option(Ball::new(2.0), Isometry3::identity());

    let p1 = Isometry3::from_constructors(&[0.0; 3], &[0.0; 3]);
    let p2 = Isometry3::from_constructors(&[0.0; 3], &[0.0; 3]);

    let q = ParryContactQry::new(2.0, ParryQryShapeType::new_standard(ParryShapeRep::BoundingSphere));
    for _ in 0..1000 {
        let res = q.query(&s1, &s2, &p1, &p2);
        println!("{:?}", res.aux_data());
    }

    let bs1 = OParryBoundingSphere { radius: 1.0, offset: Isometry3::identity() };
    let bs2 = OParryBoundingSphere { radius: 1.0, offset: Isometry3::identity() };
    println!("{:?}", bs1.intersect(&bs2, &Isometry3::identity(), &Isometry3::identity(), &()));


}