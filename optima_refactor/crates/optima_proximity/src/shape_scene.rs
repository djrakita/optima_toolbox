use std::borrow::Cow;
use ad_trait::AD;
use as_any::AsAny;
use serde::{Deserialize, Serialize};
use serde_with::serde_as;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategory};
use crate::pair_group_queries::{PairSkipsTrait, SkipReason};
use crate::shapes::OParryShape;
use optima_3d_spatial::optima_3d_pose::SerdeO3DPose;
use optima_file::traits::{FromJsonString, ToJsonString};
use optima_universal_hashmap::AHashMapWrapper;

pub trait ShapeSceneTrait<T: AD, P: O3DPose<T>> {
    type ShapeType : AsAny;
    type GetPosesInput : Clone;
    type PairSkipsType: PairSkipsTrait;

    fn get_shapes(&self) -> &Vec<Self::ShapeType>;
    fn get_shape_poses<'a>(&'a self, input: &'a Self::GetPosesInput) -> Cow<'a, Vec<P>>;
    fn sample_pseudorandom_input(&self) -> Self::GetPosesInput;
    fn get_pair_skips(&self) -> &Self::PairSkipsType;
    fn shape_id_to_shape_str(&self, id: u64) -> String;
}

#[serde_as]
#[derive(Clone, Serialize, Deserialize)]
pub struct OParryGenericShapeScene<T: AD, P: O3DPose<T>> {
    #[serde(deserialize_with="Vec::<OParryShape::<T, P>>::deserialize")]
    shapes: Vec<OParryShape<T, P>>,
    #[serde_as(as = "Vec::<SerdeO3DPose<T, P>>")]
    poses: Vec<P>,
    pair_skips: ()
}
impl<T: AD, P: O3DPose<T>> OParryGenericShapeScene<T, P> {
    pub fn new_empty() -> Self {
        Self {
            shapes: vec![],
            poses: vec![],
            pair_skips: (),
        }
    }
    pub fn new(shapes: Vec<OParryShape<T, P>>, poses: Vec<P>) -> Self {
        assert_eq!(shapes.len(), poses.len());
        Self { shapes, poses, pair_skips: () }
    }
    pub fn add_shape(&mut self, shape: OParryShape<T, P>, pose: P) {
        self.shapes.push(shape);
        self.poses.push(pose);
    }
    #[inline(always)]
    pub fn update_pose(&mut self, idx: usize, pose: P) {
        self.poses[idx] = pose;
    }
    pub fn to_other_ad_type<T1: AD>(&self) -> OParryGenericShapeScene<T1, <P::Category as O3DPoseCategory>::P<T1>> {
        self.to_other_generic_types::<T1, P::Category>()
    }
    pub fn to_other_generic_types<T1: AD, C1: O3DPoseCategory>(&self) -> OParryGenericShapeScene<T1, C1::P<T1>> {
        let json_str = self.to_json_string();
        OParryGenericShapeScene::<T1, C1::P<T1>>::from_json_string(&json_str)
    }
}
impl<T: AD, P: O3DPose<T>> ShapeSceneTrait<T, P> for OParryGenericShapeScene<T, P> {
    type ShapeType = OParryShape<T, P>;
    type GetPosesInput = ();
    type PairSkipsType = ();

    fn get_shapes(&self) -> &Vec<Self::ShapeType> {
        &self.shapes
    }

    fn get_shape_poses(&self, _input: &Self::GetPosesInput) -> Cow<Vec<P>> {
        Cow::Borrowed(&self.poses)
    }

    fn sample_pseudorandom_input(&self) -> Self::GetPosesInput {
        ()
    }

    fn get_pair_skips(&self) -> &Self::PairSkipsType {
        &self.pair_skips
    }

    fn shape_id_to_shape_str(&self, _id: u64) -> String {
        "".to_string()
    }
}


pub fn get_shape_skips_for_two_shape_scenes() -> AHashMapWrapper<(u64, u64), Vec<SkipReason>> {
    todo!()
}

pub fn get_shape_average_distances_for_two_parry_generic_shape_scenes() {
    todo!()
}
