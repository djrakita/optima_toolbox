use parry_ad::na::{Isometry3, Vector3};
use parry_ad::shape::Cuboid;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_3d_spatial::optima_3d_vec::{O3DVec, O3DVecCategoryVector3};
use optima_proximity::pair_queries::{OPairQryTrait, ParryContactQry, ParryDisMode, ParryDistanceLowerBoundQry, ParryProximaDistanceBoundsArgs, ParryProximaDistanceBoundsOutput, ParryProximaDistanceBoundsQry, ParryProximaDistanceLowerBoundArgs, ParryProximaDistanceLowerBoundQry, ParryProximaDistanceUpperBoundArgs, ParryProximaDistanceUpperBoundQry, ParryQryShapeType, ParryShapeRep};
use optima_proximity::proxima::ParryProximaQry;
use optima_proximity::shape_queries::{ContactOutputTrait, DistanceBoundsOutputTrait, DistanceLowerBoundOutputTrait, DistanceUpperBoundOutputTrait};
use optima_proximity::shapes::OParryShape;

fn main() {
    let shape_a = OParryShape::new_default(Cuboid::new(Vector3::new(1., 1., 1.)), Isometry3::identity());
    let shape_b = OParryShape::new_default(Cuboid::new(Vector3::new(1., 1., 1.)), Isometry3::identity());

    let pose_a = Isometry3::from_constructors(&[0.,0.,0.], &[0.,0.,0.]);
    let pose_b = Isometry3::from_constructors(&[5.,0.,0.], &[1.,2.,3.]);

    let c = ParryContactQry::query(&shape_a, &shape_b, &pose_a, &pose_b, &(10.0, ParryQryShapeType::Standard, ParryShapeRep::Full, None));
    let raw_dis = c.signed_distance().unwrap();
    println!("{:?}", c);

    let point1 = c.contact().unwrap().point1.o3dvec_to_other_generic_category::<f64, O3DVecCategoryVector3>();
    let point2 = c.contact().unwrap().point2.o3dvec_to_other_generic_category::<f64, O3DVecCategoryVector3>();

    println!("{:?}", point1);
    println!("{:?}", point2);

    let pose_a2 = Isometry3::from_constructors(&[0.,0.,0.], &[0.2,0.,0.1]);
    let pose_b2 = Isometry3::from_constructors(&[5.,0.1,0.1], &[1.,2.02,3.1]);

    for _ in 0..1000 {
        let p = ParryProximaDistanceBoundsQry::query(&shape_a, &shape_b, &pose_a2, &pose_b2, &ParryProximaDistanceBoundsArgs {
            parry_qry_shape_type: ParryQryShapeType::Standard,
            parry_shape_rep: ParryShapeRep::Full,
            pose_a_j: &pose_a,
            pose_b_j: &pose_b,
            closest_point_a_j: &point1,
            closest_point_b_j: &point2,
            raw_distance_j: raw_dis,
            displacement_between_a_and_b_j: &pose_a.displacement(&pose_b),
            cutoff_distance: 5.5,
            average_distance: None,
        });

        let p = p.0.unwrap();
        println!("{:?}, {:?}", p.distance_lower_bound(), p.distance_upper_bound());
        println!("{:?}", p.aux_data());
    }
}