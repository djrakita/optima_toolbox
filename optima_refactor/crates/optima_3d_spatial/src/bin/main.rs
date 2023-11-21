use std::borrow::Cow;
use ad_trait::AD;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::{ImplicitDualQuaternion, O3DPose};

fn get_cow<T: AD, P: O3DPose<T>>(p: &P) -> Cow<ImplicitDualQuaternion<T>> {
    p.o3dpose_downcast_or_convert::<ImplicitDualQuaternion<T>>()
}

fn main() {
    let p = Isometry3::from_constructors(&[1.,0.,0.], &[0., 0., 1.]);

    let res = get_cow(&p);
    println!("{:?}", res.as_ref());

}
