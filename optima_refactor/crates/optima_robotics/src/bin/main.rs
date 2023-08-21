use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion};
use optima_linalg::vecs_and_mats::NalgebraLinalg;
use optima_robotics::chain::OChain;
use optima_robotics::robotics_traits::ChainableTrait;

fn main() {
    let r = OChain::<f32, ImplicitDualQuaternion<_>, NalgebraLinalg>::from_urdf("hubo");
    let c = r.compute_compute_chain_info();

    println!("{:?}", c.link_children_joints(0));

    // let res = r.forward_kinematics(&vec![0.;6], None);
    // println!("{:?}", res);
}