use parry_ad::na::Isometry3;
use parry_ad::shape::Ball;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_proximity2::pair_group_queries::{OPairGroupQryTrait, ParryIntersectGroupQry, ParryPairSelector};
use optima_proximity2::shapes::{OParryShape, ParryShapeRep};

fn main() {
    let s1 = OParryShape::new(Ball::new(1.0), Isometry3::identity());
    let s2 = OParryShape::new(Ball::new(1.0), Isometry3::identity());
    let s3 = OParryShape::new(Ball::new(1.0), Isometry3::identity());
    let s4 = OParryShape::new(Ball::new(1.0), Isometry3::identity());

    let s = vec![s1, s2, s3, s4];

    let p1 = Isometry3::<f64>::identity();
    let p2 = Isometry3::<f64>::identity();
    let p3 = Isometry3::<f64>::identity();
    let p4 = Isometry3::<f64>::from_constructors(&[10.,0.,0.], &[0.0; 3]);

    let p = vec![p1, p2, p3, p4];

    let g = ParryIntersectGroupQry::new(ParryShapeRep::OBB, false, true);

    let o = g.query(&s, &s, &p, &p, &ParryPairSelector::HalfPairs, &());
    o.outputs().iter().for_each(|x| {
       println!("{:?}", x.output());
    });
}