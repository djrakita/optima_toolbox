use ad_trait::AD;
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPoseCategoryTrait;
use optima_linalg::OLinalgCategoryTrait;
use optima_robotics::chain::OChain;
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};
use optima_robotics::robotics_traits::AsChainTrait;

fn test<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait, AsChain: AsChainTrait<T, C, L>>(as_chain: &AsChain) {
    println!("{:?}", as_chain.as_chain());
}

fn main() {
    let mut r = ORobotDefault::<f64>::new_empty();
    r.add_chain(OChain::from_urdf("ur5"), 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    r.add_chain(OChain::from_urdf("ur5"), 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    r.add_chain(OChain::from_urdf("ur5"), 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());

    let chain = r.as_chain();
    let chain = chain.to_new_ad_type::<adfn<1>>();
    test(&chain);
}