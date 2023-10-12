use ad_trait::AD;
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPoseCategoryTrait;
use optima_linalg::OLinalgCategoryTrait;
use optima_robotics::robot::ORobot;
use optima_robotics::robot_set::ORobotSetDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};
use optima_robotics::robotics_traits::AsRobotTrait;

fn test<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait, AsChain: AsRobotTrait<T, C, L>>(as_chain: &AsChain) {
    println!("{:?}", as_chain.as_robot());
}

fn main() {
    let mut r = ORobotSetDefault::<f64>::new_empty();
    r.add_robot(ORobot::from_urdf("ur5"), 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    r.add_robot(ORobot::from_urdf("ur5"), 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    r.add_robot(ORobot::from_urdf("ur5"), 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());

    let chain = r.as_robot();
    let chain = chain.to_new_ad_type::<adfn<1>>();
    test(&chain);
}