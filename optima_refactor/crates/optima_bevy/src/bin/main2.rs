use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::chain::OChain;
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};

fn main() {
    let mut r = ORobotDefault::new_empty();
    r.add_chain(OChain::from_urdf("lite6"), 0, 0, &Isometry3::from_constructors(&[1.,0.,0.], &[0.0; 3]), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    r.add_chain(OChain::from_urdf("xarm7"), 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    r.add_chain(OChain::from_urdf("b1"), 0, 0, &Isometry3::from_constructors(&[0.,1.,0.], &[0.0; 3]), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    r.add_chain(OChain::from_urdf("z1"), 3, 0, &Isometry3::from_constructors(&[0.1,0.,0.1], &[0.0; 3]), [0.0; 3], OJointType::Fixed, OJointLimit::default());

    // println!("{:?}", r.num_dofs());

    // let mut r = r.to_new_ad_type::<adr>();

    // println!("{:?}", r.robot_as_single_chain().joints()[0]);

    r.bevy_display();
}