use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::ORobotDefault;

#[derive(Clone, Debug)]
pub struct Test {
    a: f64
}

fn main() {
    /*
    let robot = ORobotDefault::load_from_saved_robot("ur5");

    let fk_res = robot.forward_kinematics(&[0.0,0.0,1.57,1.57,1.57,0.0], None);

    let ee_pose = fk_res.get_link_pose(7).unwrap();
    println!("{:?}", ee_pose.translation);

    // [0.47470048975505724, 0.10921553768829333, 0.41995255448774055]
    */

    let t = Test { a: 1.0 };

    let option_t = &Some(t);

    let unwrapped = option_t.as_ref().unwrap();
    println!("{:?}", unwrapped);

}