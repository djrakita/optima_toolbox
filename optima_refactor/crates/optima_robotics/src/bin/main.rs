use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion, O3DPose};
use optima_linalg::vecs_and_mats::NalgebraLinalg;
use optima_robotics::chain::OChain;
use optima_robotics::robot::ORobot;
use optima_robotics::robotics_components::OJointType;
use optima_robotics::robotics_traits::ChainableTrait;

fn main() {
    let c1 = OChain::<f32, ImplicitDualQuaternion<_>, NalgebraLinalg>::from_urdf("ur5");
    let c2 = OChain::<f32, ImplicitDualQuaternion<_>, NalgebraLinalg>::from_urdf("ur5");

    let mut r = ORobot::new_empty();
    r.add_chain(c1, 0, 0, &ImplicitDualQuaternion::identity(), [0.,0.,0.], OJointType::Fixed);
    r.add_chain(c2, 0, 0, &ImplicitDualQuaternion::identity(), [0.,0.,0.], OJointType::Fixed);

    // let res = r.forward_kinematics(&vec![0.;6], None);
    // println!("{:?}", res);

    println!("{:?}", r.joints());
    println!("{:?}", r.chain_info().kinematic_hierarchy());
    println!("{:?}", r.chain_sub_dof_idxs());
    println!("{:?}", r.chain_sub_dof_idxs_range());
}