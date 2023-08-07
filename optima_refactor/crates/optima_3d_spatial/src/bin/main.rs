use ad_trait::AD;
use optima_3d_spatial::optima_se3_pose::{ImplicitDualQuaternion, OptimaSE3Pose};

fn test<'a, T: AD, P: OptimaSE3Pose<'a, T>>(pose1: &P, pose2: &P) -> P {
    pose1.mul(pose2)
}

fn main() {
    let idq1 = ImplicitDualQuaternion::new(&[0.,0.,0.], &[0.,0.,0.]);
    let idq2 = ImplicitDualQuaternion::new(&[0.,0.,0.], &[0.,0.,0.]);

    println!("{:?}", test(&idq1, &idq2));
}
