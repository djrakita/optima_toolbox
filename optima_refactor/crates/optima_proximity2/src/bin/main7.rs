use parry_ad::shape::Ball;
use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion, O3DPose};
use optima_proximity2::pair_group_queries::get_all_parry_pairs_idxs;
use optima_proximity2::shapes::OParryShape;

fn main() {
    let s1 = OParryShape::new(Ball::new(2.0_f32), ImplicitDualQuaternion::identity());
    let s2 = OParryShape::new(Ball::new(2.0_f32), ImplicitDualQuaternion::identity());
    let s3 = OParryShape::new(Ball::new(2.0_f32), ImplicitDualQuaternion::identity());

    let s = vec![s1, s2, s3];
    let i =get_all_parry_pairs_idxs(&s, &s, false, true);

    println!("{:?}", i);
}