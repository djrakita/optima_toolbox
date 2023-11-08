use std::borrow::Cow;
use ad_trait::AD;
use as_any::AsAny;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::OVec;
use crate::pair_group_queries::PairSkipsTrait;

pub trait ShapeSceneTrait<T: AD, P: O3DPose<T>> {
    type ShapeType : AsAny;
    type GetPosesInput<'a, V: OVec<T>>;
    type PairSkipsType: PairSkipsTrait;

    fn get_shapes(&self) -> &Vec<Self::ShapeType>;
    fn get_poses<'a, V: OVec<T>>(&self, input: &Self::GetPosesInput<'a, V>) -> Cow<'a, Vec<P>>;
    fn get_pair_skips(&self) -> &Self::PairSkipsType;
    fn shape_id_to_shape_str(&self, id: u64) -> String;
}