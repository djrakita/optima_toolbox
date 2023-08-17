use ad_trait::forward_ad::adf::adf_f32x4;
use ad_trait::forward_ad::adfn::adfn;
use optima_file::traits::{FromJsonString, ToJsonString};
use optima_robotics::robot_model::ORobotModelDefault;

fn main() {
    let r = ORobotModelDefault::<adfn<2>>::from_urdf("ur5");
    println!("{:?}", r);

    let s = r.to_json_string();

    let r2 = ORobotModelDefault::<adf_f32x4>::from_json_string(&s);
    println!("{:?}", r2);
}