
use ad_trait::differentiable_function::{FiniteDifferencing2, ForwardAD2, ForwardADMulti2, ReverseAD2};
use ad_trait::forward_ad::adf::adf_f32x8;
use optima_proximity::pair_group_queries::{OwnedParryDistanceGroupQry, OwnedParryDistanceGroupSequenceFilter, ParryDistanceGroupArgs, ParryDistanceGroupSequenceFilterArgs};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_robotics::robot::ORobotDefault;

fn main() {
    let r = ORobotDefault::load_from_saved_robot("ur5");

    let distance_threshold = 0.6;
    let fq = OwnedParryDistanceGroupSequenceFilter::new(ParryDistanceGroupSequenceFilterArgs::new(vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB], vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB], 0.6, true, ParryDisMode::ContactDis));
    let q = OwnedParryDistanceGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, true, f64::MIN));

    let db1 = r.get_ik_differentiable_block(ForwardADMulti2::<adf_f32x8>::new(), fq, q, &[0.1; 6], vec![6], distance_threshold);

    let fq = OwnedParryDistanceGroupSequenceFilter::new(ParryDistanceGroupSequenceFilterArgs::new(vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB], vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB], 0.6, true, ParryDisMode::ContactDis));
    let q = OwnedParryDistanceGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, true, f64::MIN));

    let db2 = r.get_ik_differentiable_block(ReverseAD2::new(), fq, q, &[0.1; 6], vec![6], distance_threshold);

    let fq = OwnedParryDistanceGroupSequenceFilter::new(ParryDistanceGroupSequenceFilterArgs::new(vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB], vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB], 0.6, true, ParryDisMode::ContactDis));
    let q = OwnedParryDistanceGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, true, f64::MIN));

    let db3 = r.get_ik_differentiable_block(FiniteDifferencing2::new(), fq, q, &[0.1; 6], vec![6], distance_threshold);

    let fq = OwnedParryDistanceGroupSequenceFilter::new(ParryDistanceGroupSequenceFilterArgs::new(vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB], vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB], 0.6, true, ParryDisMode::ContactDis));
    let q = OwnedParryDistanceGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, true, f64::MIN));

    let db4 = r.get_ik_differentiable_block(ForwardAD2::new(), fq, q, &[0.1; 6], vec![6], distance_threshold);

    let res1 = db1.derivative(&[0.101; 6]);
    let res2 = db2.derivative(&[0.101; 6]);
    let res3 = db3.derivative(&[0.101; 6]);
    let res4 = db4.derivative(&[0.101; 6]);

    println!("{:?}", res1);
    println!("{:?}", res2);
    println!("{:?}", res3);
    println!("{:?}", res4);

    // let o = SimpleOpEnOptimizer::new(r.get_dof_lower_bounds(), r.get_dof_upper_bounds(), 0.001);
    // let res = o.optimize_unconstrained(&[0.11; 6], &db);

}
