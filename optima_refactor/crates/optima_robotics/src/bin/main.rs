use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion};
use optima_linalg::vecs_and_mats::NalgebraLinalg;
use optima_robotics::chain::OChain;

fn main() {
    let c1 = OChain::<f32, ImplicitDualQuaternion<_>, NalgebraLinalg>::from_urdf("aliengo");
    println!("{:?}", c1.num_dofs());
}