use parry_ad::shape::Ball;
use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion, O3DPose};
use optima_proximity2::shapes::OParryShape;



fn main() {
    let mut s = OParryShape::new(Ball::new(2.0_f32), ImplicitDualQuaternion::identity());

    let res = s.resample_all_ids();
    println!("{:?}", res);

    let a = "test".to_string();

}