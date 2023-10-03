use std::fmt::Debug;
use ad_trait::AD;
use as_any::AsAny;
use optima_3d_spatial::optima_3d_pose::O3DPose;


pub trait OPairwiseShapeQueryTrait<T: AD> {
    type ShapeType : AsAny;
    type PoseType : O3DPose<T>;
    type Args : AsAny;
    type Output : AsAny + Debug + Clone + PartialEq + PartialOrd;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, args: &Self::Args) -> Self::Output;
}

pub trait OSingleShapeQueryTrait<T: AD> {
    type ShapeType : AsAny;
    type PoseType : O3DPose<T>;
    type Args : AsAny;
    type Output : AsAny + Debug + Clone + PartialEq + PartialOrd;

    fn query(shape: &Self::ShapeType, pose: &Self::PoseType, args: &Self::Args) -> Self::Output;
}

pub trait OPairwiseIntersectionQueryTrait { }
pub trait OPairwiseDistanceQueryTrait { }
pub trait OPairwiseContactQueryTrait { }

pub trait OParryPairwiseShapeQueryTrait { }
pub trait OParrySingleShapeQueryTrait { }


