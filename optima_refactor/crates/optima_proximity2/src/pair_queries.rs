use ad_trait::AD;
use as_any::AsAny;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use crate::shape_queries::{ContactOutputTrait, DistanceOutputTrait, IntersectOutputTrait, OShpQryContactTrait, OShpQryDistanceBoundsTrait, OShpQryDistanceLowerBoundTrait, OShpQryDistanceTrait, OShpQryDistanceUpperBoundTrait, OShpQryIntersectTrait};
use crate::shapes::{OParryShape, ParryContactOutput, ParryDisMode, ParryDistanceBoundsOutput, ParryDistanceLowerBoundOutput, ParryDistanceOutput, ParryDistanceUpperBoundOutput, ParryIntersectOutput, ParryQryShapeType, ParryShapeRep};

pub trait OPairQryTrait<T: AD, P: O3DPose<T>> {
    type ShapeTypeA : AsAny;
    type ShapeTypeB : AsAny;
    type Args : AsAny;
    type Output : AsAny + PartialOrd;
    fn query(&self, shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output;
}

pub trait OPairQryIntersectTrait<T: AD, P: O3DPose<T>> : OPairQryTrait<T, P, Output : IntersectOutputTrait> where Self::ShapeTypeA : OShpQryIntersectTrait<T, P, Self::ShapeTypeB> { }
pub trait OPairQryDistanceTrait<T: AD, P: O3DPose<T>> : OPairQryTrait<T, P, Output : DistanceOutputTrait<T>> where Self::ShapeTypeA : OShpQryIntersectTrait<T, P, Self::ShapeTypeB> { }
pub trait OPairQryContactTrait<T: AD, P: O3DPose<T>> : OPairQryTrait<T, P, Output : ContactOutputTrait<T>> where Self::ShapeTypeA : OShpQryIntersectTrait<T, P, Self::ShapeTypeB> { }

pub struct ParryIntersectQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryIntersectQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryQryShapeType, ParryShapeRep);
    type Output = ParryIntersectOutput;

    fn query(&self, shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        shape_a.intersect(shape_b, pose_a, pose_b, args)
    }
}
impl<T: AD, P: O3DPose<T>> OPairQryIntersectTrait<T, P> for ParryIntersectQry { }

pub struct ParryDistanceQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep);
    type Output = ParryDistanceOutput<T>;

    fn query(&self, shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        shape_a.distance(shape_b, pose_a, pose_b, args)
    }
}
impl<T: AD, P: O3DPose<T>> OPairQryDistanceTrait<T, P> for ParryDistanceQry { }

#[derive(Clone, Debug)]
pub struct ParryContactQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryContactQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (T, ParryQryShapeType, ParryShapeRep);
    type Output = ParryContactOutput<T>;

    fn query(&self, shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        shape_a.contact(shape_b, pose_a, pose_b, &args)
    }
}
impl<T: AD, P: O3DPose<T>> OPairQryContactTrait<T, P> for ParryContactQry { }

pub struct ParryDistanceLowerBoundQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceLowerBoundQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep);
    type Output = ParryDistanceLowerBoundOutput<T>;

    fn query(&self, shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        shape_a.distance_lower_bound(shape_b, pose_a, pose_b, args)
    }
}

pub struct ParryDistanceUpperBoundQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceUpperBoundQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep);
    type Output = ParryDistanceUpperBoundOutput<T>;

    fn query(&self, shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        shape_a.distance_upper_bound(shape_b, pose_a, pose_b, args)
    }
}

pub struct ParryDistanceBoundsQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceBoundsQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep);
    type Output = ParryDistanceBoundsOutput<T>;

    fn query(&self, shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        shape_a.distance_bounds(shape_b, pose_a, pose_b, &args)
    }
}
