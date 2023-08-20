use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::vecs_and_mats::OVec;

pub trait OForwardKinematicsTrait<T: AD, P: O3DPose<T>> {
    fn forward_kinematics<V: OVec<T>>(&self, state: &V, base_offset: Option<&P>) -> Vec<Option<P>>;
    fn forward_kinematics_floating_chain<V: OVec<T>>(&self, state: &V, base_offset: Option<&P>) -> Vec<Option<P>>;
}

