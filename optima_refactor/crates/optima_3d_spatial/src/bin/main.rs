use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion, O3DPose};

fn main() {
    let p = ImplicitDualQuaternion::<f64>::identity();
    let r = p.as_any().downcast_ref::<ImplicitDualQuaternion<f64>>();
    println!("{:?}", r);
}
