use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryImplicitDualQuaternion};
use optima_file::path::OAssetLocation::RobotSet;
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robot_set::ORobotSetDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};
use optima_robotics::robotics_traits::AsRobotTrait;

fn main()  {
    let r1 = ORobotDefault::from_urdf("ur5");
    let r2 = ORobotDefault::from_urdf("lite6");

    let mut robot_set = ORobotSetDefault::new_empty();
    robot_set.add_robot(r1, 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());
    robot_set.add_robot(r2, 0, 0, &Isometry3::identity(), [0.0; 3], OJointType::Fixed, OJointLimit::default());

    let robot = robot_set.as_robot();

    let res = robot.forward_kinematics(&[0.0; 12], None);

    // robot.links().iter().for_each(|x| {
       // println!("{:?}, {:?}", x.link_idx(), x.name());
    // });

    let link_20_pose = res.get_link_pose(20);

    println!("{:?}", link_20_pose);
}