use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::chain::OChainDefault;

fn main() {
    /*
    let mut robot = ORobotDefault::<f64>::new_empty();
    let chain = OChainDefault::<f64>::from_urdf("b1");
    robot.add_chain(chain, 0, 0, &Isometry3::identity(), [0.;3], OJointType::Floating, OJointLimit::default());
    let chain = OChainDefault::<f64>::from_urdf("z1");
    robot.add_chain(chain, 1, 1, &Isometry3::from_constructors(&[0.195,0.,0.07], &[0.,0.,0.]), [0.,0.,0.], OJointType::Fixed, OJointLimit::default());
    // robot.bevy_display();

    test_script(robot);
    */
    let c = OChainDefault::<f64>::from_urdf("b1");
    c.bevy_display();
}