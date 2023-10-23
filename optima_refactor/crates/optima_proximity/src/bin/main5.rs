use parry_ad::na::{Isometry3, Vector3};
use parry_ad::shape::Cuboid;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_proximity::parry::parry_group_queries::{OParryPairCullerCategory, OParryPairShpGroupQryDefault};
use optima_proximity::parry::parry_queries::{OParryDisQry, OParryDisQryArgs, ParryShapeRepresentation};
use optima_proximity::parry::parry_shapes::OParryShp;
use optima_proximity::shape_group_query_traits::{OPairShpGroupQryTrait, OShpGroupPairSelector};

fn main() {
    let shape1 = OParryShp::<f64, Isometry3<f64>>::new(Cuboid::new(Vector3::new(1., 1., 1.)), None);
    let shape2 = OParryShp::<f64, Isometry3<f64>>::new(Cuboid::new(Vector3::new(1., 1., 1.)), None);
    let shape3 = OParryShp::<f64, Isometry3<f64>>::new(Cuboid::new(Vector3::new(1., 1., 1.)), None);

    let pose1 = Isometry3::from_constructors(&[1.,0.,0.], &[0.,0.,0.]);
    let pose2 = Isometry3::from_constructors(&[1.,3.,0.], &[0.,0.,0.]);
    let pose3 = Isometry3::from_constructors(&[1.,0.,1.9], &[0.,0.,0.]);
    
    let shapes = vec![shape1,shape2,shape3];
    let poses = vec![pose1,pose2,pose3];

    let cullers = vec![
        OParryPairCullerCategory::Intersect { rep: ParryShapeRepresentation::BoundingSphere }
    ];

    let o = OParryPairShpGroupQryDefault::<f64, OParryDisQry, ()>::new(true, cullers, OParryDisQryArgs { rep_a: ParryShapeRepresentation::Full, rep_b: ParryShapeRepresentation::Full });

    let res = o.query(&shapes, &shapes, &poses, &poses, OShpGroupPairSelector::HalfPairs);

    res.iter().for_each(|x| {
       println!("{:?}", (x.output, x.shape_indices))
    });
}