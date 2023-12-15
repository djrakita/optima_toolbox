use parry_ad::na::{Isometry3, Vector3};
use parry_ad::shape::Cuboid;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_proximity::pair_group_queries::{OwnedParryDistanceGroupQry, ParryDistanceGroupArgs, ParryPairSelector, ProximityLossFunctionHinge, ToParryProximityOutputTrait};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity::proxima::{OwnedParryProximaQry, PairGroupQryArgsParryProxima, ProximaTermination};
use optima_proximity::shapes::OParryShape;

fn main() {
    let q1 = OwnedParryProximaQry::new(PairGroupQryArgsParryProxima::new(ParryShapeRep::Full, false, false, ProximaTermination::MaxError(0.5), 15.0, 15.0));

    let q2 = OwnedParryDistanceGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, false, false, f64::MIN));

    let s1 = OParryShape::new(Cuboid::new(Vector3::new(0.5,1.,0.1)), Isometry3::identity());
    let s2 = OParryShape::new(Cuboid::new(Vector3::new(0.5,1.,0.1)), Isometry3::identity());
    let s3 = OParryShape::new(Cuboid::new(Vector3::new(0.5,1.,0.1)), Isometry3::identity());
    let s4 = OParryShape::new(Cuboid::new(Vector3::new(0.5,1.,0.1)), Isometry3::identity());

    let shapes = vec![s1, s2, s3, s4];

    let p1 = Isometry3::from_constructors(&[0.,0.,0.], &[0.3,0.2,0.1]);
    let p2 = Isometry3::from_constructors(&[3.,5.,0.7], &[0.4,0.9,0.3]);
    let p3 = Isometry3::from_constructors(&[0.3,4.,0.2], &[0.5,0.8,0.1]);
    let p4 = Isometry3::from_constructors(&[1.,0.,10.], &[0.6,0.7,0.1]);

    let mut poses = vec![p1, p2, p3, p4];

    let res = q2.query(&shapes, &shapes, &poses, &poses, &ParryPairSelector::HalfPairs, &(), &());
    let p = res.get_proximity_objective_value(15.0, 15.0, ProximityLossFunctionHinge);
    println!("{:?}", res.aux_data());
    println!("{:?}", p);

    let res = q1.query(&shapes, &shapes, &poses, &poses, &ParryPairSelector::HalfPairs, &(), &());
    let p = res.get_proximity_objective_value(15.0, 15.0, ProximityLossFunctionHinge);
    println!("{:?}", res.aux_data());
    println!("{:?}", res.maximum_possible_proximity_value_error());
    println!("{:?}", p);

    poses[0] = Isometry3::from_constructors(&[0.,0.,0.], &[0.3,0.2,0.11]);

    let res = q2.query(&shapes, &shapes, &poses, &poses, &ParryPairSelector::HalfPairs, &(), &());
    let p = res.get_proximity_objective_value(15.0, 15.0, ProximityLossFunctionHinge);
    println!("{:?}", res.aux_data());
    println!("{:?}", p);

    let res = q1.query(&shapes, &shapes, &poses, &poses, &ParryPairSelector::HalfPairs, &(), &());
    let p = res.get_proximity_objective_value(15.0, 15.0, ProximityLossFunctionHinge);
    println!("{:?}", res.aux_data());
    println!("{:?}", res.maximum_possible_proximity_value_error());
    println!("{:?}", p);

    poses[0] = Isometry3::from_constructors(&[0.,0.,0.], &[0.3,0.19,0.11]);

    let res = q2.query(&shapes, &shapes, &poses, &poses, &ParryPairSelector::HalfPairs, &(), &());
    let p = res.get_proximity_objective_value(15.0, 15.0, ProximityLossFunctionHinge);
    println!("{:?}", res.aux_data());
    println!("{:?}", p);

    let res = q1.query(&shapes, &shapes, &poses, &poses, &ParryPairSelector::HalfPairs, &(), &());
    let p = res.get_proximity_objective_value(15.0, 15.0, ProximityLossFunctionHinge);
    println!("{:?}", res.aux_data());
    println!("{:?}", res.maximum_possible_proximity_value_error());
    println!("{:?}", p);
}