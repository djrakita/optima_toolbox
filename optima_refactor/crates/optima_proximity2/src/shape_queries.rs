use ad_trait::AD;
use as_any::AsAny;
use optima_3d_spatial::optima_3d_pose::{O3DPose};

pub trait OShpQryIntersectTrait<T: AD, P: O3DPose<T>, B : AsAny> {
    type Args : AsAny;
    type Output : AsAny + IntersectOutputTrait;
    fn intersect(&self, other: &B, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output;
}
pub trait OShpQryDistanceTrait<T: AD, P: O3DPose<T>, B : AsAny> {
    type Args : AsAny;
    type Output : AsAny + DistanceOutputTrait<T>;
    fn distance(&self, other: &B, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output;
}
pub trait OShpQryContactTrait<T: AD, P: O3DPose<T>, B: AsAny> {
    type Args : AsAny;
    type Output : AsAny + ContactOutputTrait<T>;
    fn contact(&self, other: &B, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output;
}

pub trait IntersectOutputTrait : PartialOrd {
    fn intersect(&self) -> bool;
}
pub trait DistanceOutputTrait<T: AD> : PartialOrd {
    fn distance(&self) -> T;
}
pub trait ContactOutputTrait<T: AD> : PartialOrd {
    fn signed_distance(&self) -> Option<T>;
}
