use nalgebra::Isometry3;
use optima_robotics::chain::OChain;
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};

fn main() {
    let mut r = ORobotDefault::<f64>::new_from_single_chain_name("ur5");
    r.add_chain(OChain::from_urdf("ur5"), 1, 9, &Isometry3::identity(), [1.,0.,0.], OJointType::Continuous, OJointLimit::default());

    let c = r.robot_as_single_chain();

    println!("{:?}", c);
}