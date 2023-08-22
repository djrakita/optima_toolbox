use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion, O3DPose};
use optima_linalg::vecs_and_mats::NalgebraLinalg;
use optima_robotics::chain::OChain;
use optima_robotics::robot::ORobot;
use optima_robotics::robotics_components::OJointType;
use optima_robotics::robotics_traits::JointTrait;

fn main() {
    let c1 = OChain::<f32, ImplicitDualQuaternion<_>, NalgebraLinalg>::from_urdf("ur5");
    let c2 = OChain::<f32, ImplicitDualQuaternion<_>, NalgebraLinalg>::from_urdf("ur5");

    let mut r = ORobot::new_empty();
    r.add_chain(c1, 0, 0, &ImplicitDualQuaternion::identity(), [0.,0.,1.], OJointType::Fixed);
    r.add_chain(c2, 0, 0, &ImplicitDualQuaternion::from_translation_and_rotation_constructor(&[0.,0.,0.], &[0.0; 3]), [0.,0.,1.], OJointType::Fixed);

    r.macro_joints().iter().for_each(|x| println!(" > {:?}", x.sub_dof_idxs_range() ));
    r.chain_wrappers().iter().for_each(|x| println!("{:?}", x.sub_dof_idxs_range() ));

    println!("{:?}", r.get_macro_joint_variable_transform(&[1.0; 20], 1));
}