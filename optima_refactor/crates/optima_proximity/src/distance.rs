use ad_trait::AD;
use parry_ad::na::{Isometry3};
use parry_ad::query;
use optima_3d_spatial::optima_3d_pose::{O3DPose};
use crate::ParryShapeWrapper;

pub trait ODistanceCategoryTrait {
    type D<T: AD> : ODistanceTrait<T>;
}

pub trait ODistanceTrait<T: AD> {
    fn distance<P: O3DPose<T>>(&self, self_pose: &P, other: &Self, other_pose: &P) -> T;
}

impl<T: AD> ODistanceTrait<T> for ParryShapeWrapper<T> {
    fn distance<P: O3DPose<T>>(&self, self_pose: &P, other: &Self, other_pose: &P) -> T {
        if let (Some(self_pose), Some(other_pose)) = (self_pose.as_any().downcast_ref::<Isometry3<T>>(), other_pose.as_any().downcast_ref::<Isometry3<T>>()) {
            query::distance(self_pose, &**self.shape(), other_pose, &**other.shape()).expect("error")
        } else {
            let self_pose = Isometry3::from_translation_and_rotation(self_pose.translation(), self_pose.rotation());
            let other_pose = Isometry3::from_translation_and_rotation(other_pose.translation(), other_pose.rotation());
            query::distance(&self_pose, &**self.shape(), &other_pose, &**other.shape()).expect("error")
        }
    }
}

