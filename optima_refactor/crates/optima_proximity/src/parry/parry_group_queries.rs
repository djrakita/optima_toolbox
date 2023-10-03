use std::marker::PhantomData;
use ad_trait::AD;
use crate::shape_group_query_traits::{OPairwiseShapeGroupQuery, OPairwiseShapeGroupQueryElement, OShapeGroupPairwiseSelector};
use crate::shape_query_traits::{OPairwiseShapeQueryTrait, OParryPairwiseShapeQueryTrait};

pub struct OParryPairwiseShapeGroupQueryDefaultIteration<T: AD, Q: OPairwiseShapeQueryTrait<T> + OParryPairwiseShapeQueryTrait>(PhantomData<(T, Q)>);
pub struct OParryPairwiseShapeGroupQueryDefaultIterationArgs<T: AD, Q: OPairwiseShapeQueryTrait<T> + OParryPairwiseShapeQueryTrait> {
    pub sorted_outputs: bool,
    pub args: Q::Args
}
impl<T: AD, Q: OPairwiseShapeQueryTrait<T> + OParryPairwiseShapeQueryTrait> OParryPairwiseShapeGroupQueryDefaultIteration<T, Q> {
    pub fn new() -> Self { Self(PhantomData::default()) }
}
impl<T: AD, Q: OPairwiseShapeQueryTrait<T> + OParryPairwiseShapeQueryTrait + 'static> OPairwiseShapeGroupQuery<T> for OParryPairwiseShapeGroupQueryDefaultIteration<T, Q> {
    type ShapeType = Q::ShapeType;
    type PoseType = Q::PoseType;
    type Args = OParryPairwiseShapeGroupQueryDefaultIterationArgs<T, Q>;
    type Output = Vec<OPairwiseShapeGroupQueryElement<Q::Output>>;

    fn query(shape_group_a: &Vec<&Self::ShapeType>, shape_group_b: &Vec<&Self::ShapeType>, poses_a: &Vec<Self::PoseType>, poses_b: &Vec<Self::PoseType>, selector: OShapeGroupPairwiseSelector, args: &Self::Args) -> Self::Output {
        assert_eq!(shape_group_a.len(), poses_a.len());
        assert_eq!(shape_group_b.len(), poses_b.len());

        let mut out = vec![];

        match &selector {
            OShapeGroupPairwiseSelector::AllPairs => {
                shape_group_a.iter().zip(poses_a.iter()).enumerate().for_each(|(i, (shape_a, pose_a))| {
                    shape_group_b.iter().zip(poses_b.iter()).enumerate().for_each(|(j, (shape_b, pose_b))| {
                        let output = Q::query(shape_a, shape_b, pose_a, pose_b, &args.args);
                        out.push( OPairwiseShapeGroupQueryElement { shape_indices: (i, j), output } );
                    });
                });
            }
            OShapeGroupPairwiseSelector::HalfPairs => {
                shape_group_a.iter().zip(poses_a.iter()).enumerate().for_each(|(i, (shape_a, pose_a))| {
                    shape_group_b.iter().zip(poses_b.iter()).enumerate().for_each(|(j, (shape_b, pose_b))| {
                        if i < j {
                            let output = Q::query(shape_a, shape_b, pose_a, pose_b, &args.args);
                            out.push(OPairwiseShapeGroupQueryElement { shape_indices: (i, j), output });
                        }
                    });
                });
            }
            OShapeGroupPairwiseSelector::GivenPairsByIdxs(idxs) => {
                idxs.iter().for_each(|(i, j)| {
                    let shape_a = shape_group_a[*i];
                    let shape_b = shape_group_b[*j];
                    let pose_a = &poses_a[*i];
                    let pose_b = &poses_b[*i];
                    let output = Q::query(shape_a, shape_b, pose_a, pose_b, &args.args);
                    out.push(OPairwiseShapeGroupQueryElement { shape_indices: (*i, *j), output });
                });
            }
        }

        if args.sorted_outputs { out.sort(); }

        out
    }
}