use optima_robotics::chain::OChainDefault;

fn main() {
    let c = OChainDefault::from_urdf("ur5");

    println!("{:?}", c);
}