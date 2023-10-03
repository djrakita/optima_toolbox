use std::cmp::Ordering;
use std::fmt::Debug;
use ad_trait::AD;
use as_any::AsAny;
use optima_3d_spatial::optima_3d_pose::O3DPose;

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug)]
pub enum OShapeGroupPairwiseSelector {
    AllPairs,
    /// This is useful when shape_group_a and shape_group_b are the same, i.e., no need to redo pairwise checks twice
    HalfPairs,
    GivenPairsByIdxs(Vec<(usize, usize)>)
}

pub trait OPairwiseShapeGroupQuery<T: AD> {
    type ShapeType : AsAny;
    type PoseType : O3DPose<T>;
    type Args : AsAny;
    type Output : AsAny;

    fn query(shape_group_a: &Vec<&Self::ShapeType>,
             shape_group_b: &Vec<&Self::ShapeType>,
             poses_a: &Vec<Self::PoseType>,
             poses_b: &Vec<Self::PoseType>,
             selector: OShapeGroupPairwiseSelector,
             args: &Self::Args) -> Self::Output;
}

pub trait OSingleShapeGroupQuery<T: AD> {
    type ShapeType : AsAny;
    type PoseType : O3DPose<T>;
    type Args : AsAny;
    type Output : AsAny;

    fn query(shape_group: &Vec<&Self::ShapeType>,
             poses: &Vec<Self::PoseType>,
             selector: OShapeGroupPairwiseSelector,
             args: &Self::Args) -> Self::Output;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct OPairwiseShapeGroupQueryElement<O: Clone + Debug + PartialEq + PartialOrd> {
    pub shape_indices: (usize, usize),
    pub output: O
}
impl<O: Clone + Debug + PartialEq + PartialOrd> PartialEq for OPairwiseShapeGroupQueryElement<O> {
    fn eq(&self, other: &Self) -> bool {
        self.output.eq(&other.output)
    }
}
impl<O: Clone + Debug + PartialEq + PartialOrd> PartialOrd for OPairwiseShapeGroupQueryElement<O> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.output.partial_cmp(&other.output)
    }
}
impl<O: Clone + Debug + PartialEq + PartialOrd> Eq for OPairwiseShapeGroupQueryElement<O> { }
impl<O: Clone + Debug + PartialEq + PartialOrd> Ord for OPairwiseShapeGroupQueryElement<O> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(&other).expect("error")
    }
}

pub struct OSingleShapeGroupQueryElement<O: Clone + Debug + PartialEq + PartialOrd> {
    pub shape_idx: usize,
    pub output: O
}
impl<O: Clone + Debug + PartialEq + PartialOrd> PartialEq for OSingleShapeGroupQueryElement<O> {
    fn eq(&self, other: &Self) -> bool {
        self.output.eq(&other.output)
    }
}
impl<O: Clone + Debug + PartialEq + PartialOrd> PartialOrd for OSingleShapeGroupQueryElement<O> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.output.partial_cmp(&other.output)
    }
}
impl<O: Clone + Debug + PartialEq + PartialOrd> Eq for OSingleShapeGroupQueryElement<O> { }
impl<O: Clone + Debug + PartialEq + PartialOrd> Ord for OSingleShapeGroupQueryElement<O> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(&other).expect("error")
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////



