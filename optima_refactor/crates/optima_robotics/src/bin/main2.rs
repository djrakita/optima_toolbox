use optima_3d_spatial::optima_3d_pose::O3DPoseCategoryIsometry3;
use optima_linalg::OLinalgCategoryNDarray;
use optima_robotics::robot::ORobot;

fn main() {
    let r = ORobot::<f64, O3DPoseCategoryIsometry3, OLinalgCategoryNDarray>::new_from_single_chain_name("b1");
    println!("{:?}", r);
}