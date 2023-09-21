use nalgebra::Isometry3;
use optima_robotics::chain::OChain;
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};

fn main() {
    let mut r = ORobotDefault::<f64>::new_empty();
    r.add_chain(OChain::from_urdf("ur5"), 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    r.add_chain(OChain::from_urdf("ur5"), 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    r.add_chain(OChain::from_urdf("ur5"), 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());

    let c = r.robot_as_single_chain();

    println!("{:?}", c);
}