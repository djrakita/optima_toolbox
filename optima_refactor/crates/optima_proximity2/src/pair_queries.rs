use ad_trait::AD;
use as_any::AsAny;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use crate::shape_queries::{ContactOutputTrait, DistanceOutputTrait, IntersectOutputTrait, OShpQryContactTrait, OShpQryDistanceTrait, OShpQryIntersectTrait};
use crate::shapes::{OParryShape, ParryContactOutput, ParryDisMode, ParryDistanceOutput, ParryIntersectOutput, ParryQryShapeType};

pub trait OPairQryTrait<T: AD, P: O3DPose<T>> {
    type ShapeTypeA : AsAny;
    type ShapeTypeB : AsAny;
    type Output : AsAny + PartialOrd;
    fn query(&self, shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P) -> Self::Output;
}

pub trait OPairQryIntersectTrait<T: AD, P: O3DPose<T>> : OPairQryTrait<T, P, Output : IntersectOutputTrait> where Self::ShapeTypeA : OShpQryIntersectTrait<T, P, Self::ShapeTypeB> { }
pub trait OPairQryDistanceTrait<T: AD, P: O3DPose<T>> : OPairQryTrait<T, P, Output : DistanceOutputTrait<T>> where Self::ShapeTypeA : OShpQryIntersectTrait<T, P, Self::ShapeTypeB> { }
pub trait OPairQryContactTrait<T: AD, P: O3DPose<T>> : OPairQryTrait<T, P, Output : ContactOutputTrait<T>> where Self::ShapeTypeA : OShpQryIntersectTrait<T, P, Self::ShapeTypeB> { }

#[derive(Clone, Debug)]
pub struct ParryIntersectQry {
    parry_qry_shape_type: ParryQryShapeType
}
impl ParryIntersectQry {
    pub fn new(parry_qry_shape_type: ParryQryShapeType) -> Self {
        Self { parry_qry_shape_type }
    }
}
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryIntersectQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Output = ParryIntersectOutput;

    fn query(&self, shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P) -> Self::Output {
        shape_a.intersect(shape_b, pose_a, pose_b, &self.parry_qry_shape_type)
    }
}
impl<T: AD, P: O3DPose<T>> OPairQryIntersectTrait<T, P> for ParryIntersectQry { }

#[derive(Clone, Debug)]
pub struct ParryDistanceQry {
    parry_qry_shape_type: ParryQryShapeType,
    parry_dis_mode: ParryDisMode
}
impl ParryDistanceQry {
    pub fn new(parry_qry_shape_type: ParryQryShapeType, parry_dis_mode: ParryDisMode) -> Self {
        Self {
            parry_qry_shape_type,
            parry_dis_mode,
        }
    }
}
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Output = ParryDistanceOutput<T>;

    fn query(&self, shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P) -> Self::Output {
        shape_a.distance(shape_b, pose_a, pose_b, &(self.parry_dis_mode.clone(), self.parry_qry_shape_type.clone()))
    }
}
impl<T: AD, P: O3DPose<T>> OPairQryDistanceTrait<T, P> for ParryDistanceQry { }

#[derive(Clone, Debug)]
pub struct ParryContactQry<T: AD> {
    margin: T,
    parry_qry_shape_type: ParryQryShapeType
}
impl<T: AD> ParryContactQry<T> {
    pub fn new(margin: T, parry_qry_shape_type: ParryQryShapeType) -> Self {
        Self {
            margin,
            parry_qry_shape_type
        }
    }
}
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryContactQry<T> {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Output = ParryContactOutput<T>;

    fn query(&self, shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P) -> Self::Output {
        shape_a.contact(shape_b, pose_a, pose_b, &(self.margin, self.parry_qry_shape_type.clone()))
    }
}
impl<T: AD, P: O3DPose<T>> OPairQryContactTrait<T, P> for ParryContactQry<T> { }
