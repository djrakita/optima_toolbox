use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::chain::OChainDefault;

fn main() {
    let chain = OChainDefault::from_urdf("b1");
    chain.bevy_display();
}