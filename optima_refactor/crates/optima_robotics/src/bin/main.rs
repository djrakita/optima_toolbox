use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion, O3DPose};
use optima_linalg::vecs_and_mats::NalgebraLinalg;
use optima_robotics::chain::OChain;
use optima_robotics::robot::ORobot;
use optima_robotics::robotics_components::{OJointLimit, OJointType};
use optima_robotics::robotics_traits::JointTrait;

fn main() {
    let c1 = OChain::<f64, ImplicitDualQuaternion<_>, NalgebraLinalg>::from_urdf("ur5");
    let c2 = OChain::<f64, ImplicitDualQuaternion<_>, NalgebraLinalg>::from_urdf("ur5");

    let mut r = ORobot::new_empty();
    r.add_chain(c1, 0, 0, &ImplicitDualQuaternion::identity(), [0.,0.,1.], OJointType::Floating, OJointLimit::default());
    r.add_chain(c2, 0, 0, &ImplicitDualQuaternion::from_translation_and_rotation_constructor(&[0.,0.,0.], &[0.0; 3]), [0.,0.,1.], OJointType::Floating, OJointLimit::default());

    r.dof_to_joint_and_sub_dof_idxs().iter().for_each(|x| println!("{:?}", x));
}