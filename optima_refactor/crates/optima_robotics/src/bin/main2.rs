use optima_robotics::robot::ORobotDefault;

fn main() {
    let c = ORobotDefault::from_urdf("ur5");

    println!("{:?}", c);
}