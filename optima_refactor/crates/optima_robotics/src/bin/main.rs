use optima_robotics::robot_model::ORobotModelDefault;

fn main() {
    let mut r = ORobotModelDefault::new_empty();
    r.add_from_urdf("ur5");
    r.joints().iter().for_each(|x| {
       println!("{:?}", x);
    });
}