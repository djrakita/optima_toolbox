use optima_robotics::robot_model::{ORobotModel};
use optima_utils::arr_storage::VecStor;
use nalgebra::Isometry3;

fn main() {
    let r = ORobotModel::<f64, Isometry3<_>, VecStor, 100, 100>::from_urdf("ur5");

    r.joints().iter().for_each(|x| {
       println!("{:?}", x);
    });
}