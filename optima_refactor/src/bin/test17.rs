
use optima_bevy::optima_bevy_utils::robotics::{BevyRoboticsTrait, BevyRoboticsTraitF64};
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_optimization::robotics_optimization_functions::{AxisDirection, LookAtTarget};

fn main() {
    let robot = ORobotDefault::load_from_saved_robot("xarm7_bimanual_viewpoint");
    // let db = robot.get_default_ik_lookat_differentiable_block()

    robot.bevy_ik_lookat(20, &vec![0.027424497604215992, 0.012629603411419232, -1.493325843865225, -0.09772824061682063, 0.3075457161480277, -0.025514083801439952, 0.23897005172161995, -0.07259634785166093, 0.14339207718259375, 3.4451808789359686, -1.5286903665311629, -0.09559754477684124, 0.28156203746477776, -0.1849232765059267, 0.23114614344462833, 0.01595253002723443], 32, AxisDirection::Z, AxisDirection::Y, LookAtTarget::RobotLink(20));

    // robot.bevy_display();

    // let robot = ORobotDefault::load_from_saved_robot("xarm7_with_gripper_and_rail");
    // robot.bevy_simple_ik(19, &[0.5; 9]);
}