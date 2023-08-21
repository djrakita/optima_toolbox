use optima_3d_spatial::optima_3d_pose::ImplicitDualQuaternion;
use optima_linalg::vecs_and_mats::NalgebraLinalg;
use optima_robotics::robot::ORobot;
use optima_robotics::robotics_traits::OForwardKinematicsTrait;

fn main() {
    let r = ORobot::<f32, ImplicitDualQuaternion<_>, NalgebraLinalg>::from_urdf("ur5");
    let res = r.forward_kinematics_floating_chain(&vec![0.;6], 10, 4, None);
    println!("{:?}", res);
}