use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion};
use optima_linalg::vecs_and_mats::NalgebraLinalg;
use optima_robotics::chain::OChain;
use optima_robotics::robotics_traits::ChainableTrait;

fn main() {
    let c1 = OChain::<f32, ImplicitDualQuaternion<_>, NalgebraLinalg>::from_urdf("go1");
    println!("{:?}", c1.num_dofs());

    println!("{:?}", c1.links()[1].convex_decomposition_file_paths());
}