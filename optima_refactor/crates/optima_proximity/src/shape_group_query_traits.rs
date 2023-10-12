use std::cmp::Ordering;
use std::fmt::Debug;
use ad_trait::AD;
use as_any::AsAny;
use optima_3d_spatial::optima_3d_pose::O3DPose;

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait OPairShpGroupQryTrait<T: AD> {
    type ShapeType : AsAny;
    type PoseType : O3DPose<T>;
    type Output : AsAny;

    fn query(&self,
             shape_group_a: &Vec<&Self::ShapeType>,
             shape_group_b: &Vec<&Self::ShapeType>,
             poses_a: &Vec<Self::PoseType>,
             poses_b: &Vec<Self::PoseType>,
             selector: OShpGroupPairSelector) -> Self::Output;
}

pub trait OSingleShapeGroupQryTrait<T: AD> {
    type ShapeType : AsAny;
    type PoseType : O3DPose<T>;
    type Output : AsAny;

    fn query(&self,
             shape_group: &Vec<&Self::ShapeType>,
             poses: &Vec<Self::PoseType>,
             selector: OShpGroupSingleSelector) -> Self::Output;
}

pub trait OShpGroupQryTermination<I> {
    fn terminate(input: &I) -> bool;
}
impl<I> OShpGroupQryTermination<I> for () {
    fn terminate(_input: &I) -> bool {
        false
    }
}

pub trait OPairShpGroupCullerTrait<T: AD> : OPairShpGroupQryTrait<T, Output = Vec<(usize, usize)>> { }
pub trait OSingleShpGroupCullerTrait<T: AD> : OSingleShapeGroupQryTrait<T, Output = Vec<usize>> { }

#[derive(Clone, Debug)]
pub enum OShpGroupPairSelector {
    AllPairs,
    /// This is useful when shape_group_a and shape_group_b are the same, i.e., no need to redo pairwise checks twice
    HalfPairs,
    GivenPairsByIdxs(Vec<(usize, usize)>)
}

#[derive(Clone, Debug)]
pub enum OShpGroupSingleSelector {
    All,
    GivenByIdxs(Vec<usize>)
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct OPairShpGroupQryElement<O: Clone + Debug + PartialEq + PartialOrd> {
    pub shape_indices: (usize, usize),
    pub output: O
}
impl<O: Clone + Debug + PartialEq + PartialOrd> PartialEq for OPairShpGroupQryElement<O> {
    fn eq(&self, other: &Self) -> bool {
        self.output.eq(&other.output)
    }
}
impl<O: Clone + Debug + PartialEq + PartialOrd> PartialOrd for OPairShpGroupQryElement<O> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.output.partial_cmp(&other.output)
    }
}
impl<O: Clone + Debug + PartialEq + PartialOrd> Eq for OPairShpGroupQryElement<O> { }
impl<O: Clone + Debug + PartialEq + PartialOrd> Ord for OPairShpGroupQryElement<O> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(&other).expect("error")
    }
}

pub struct OSingleShpGroupQryElement<O: Clone + Debug + PartialEq + PartialOrd> {
    pub shape_idx: usize,
    pub output: O
}
impl<O: Clone + Debug + PartialEq + PartialOrd> PartialEq for OSingleShpGroupQryElement<O> {
    fn eq(&self, other: &Self) -> bool {
        self.output.eq(&other.output)
    }
}
impl<O: Clone + Debug + PartialEq + PartialOrd> PartialOrd for OSingleShpGroupQryElement<O> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.output.partial_cmp(&other.output)
    }
}
impl<O: Clone + Debug + PartialEq + PartialOrd> Eq for OSingleShpGroupQryElement<O> { }
impl<O: Clone + Debug + PartialEq + PartialOrd> Ord for OSingleShpGroupQryElement<O> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.partial_cmp(&other).expect("error")
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////





