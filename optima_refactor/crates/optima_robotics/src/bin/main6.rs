use nalgebra::Isometry3;
use optima_robotics::robot::{ORobotDefault, SaveRobot};
use optima_robotics::robot_set::ORobotSetDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};
use optima_robotics::robotics_traits::AsRobotTrait;

fn main() {
    // let mut r = ORobotDefault::from_urdf("b1");
    // r.preprocess(SaveRobot::Save(None), None, None);

    let r1 = ORobotDefault::load_from_saved_robot("b1");
    let r2 = ORobotDefault::load_from_saved_robot("b1");

    let mut r = ORobotSetDefault::new_empty();
    r.add_robot(r1, 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    r.add_robot(r2, 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());

    let mut rr = r.as_robot().to_owned();
    rr.preprocess(SaveRobot::DoNotSave, Some(1000), None);
}