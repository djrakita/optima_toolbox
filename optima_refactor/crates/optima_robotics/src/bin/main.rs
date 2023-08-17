use ad_trait::forward_ad::adf::adf_f32x4;
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::ImplicitDualQuaternion;
use optima_file::traits::{FromJsonString, ToJsonString};
use optima_linalg::vecs_and_mats::NdarrayLinalg;
use optima_robotics::robot_model::{ORobotArrStorageVec, ORobotModel, ORobotModelDefault};

fn main() {
    let r = ORobotModelDefault::<adfn<2>>::from_urdf("hubo");
    println!("{:?}", r);

    let s = r.to_json_string();

    let r2 = ORobotModel::<adf_f32x4, ImplicitDualQuaternion<_>, NdarrayLinalg, ORobotArrStorageVec>::from_json_string(&s);
    println!("{:?}", r2);
}