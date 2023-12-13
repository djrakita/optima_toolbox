use std::sync::RwLock;
use ad_trait::AD;
use parry_ad::na::Isometry3;
use serde::{Deserialize, Serialize};
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_3d_spatial::optima_3d_rotation::O3DRotation;
use optima_universal_hashmap::AHashMapWrapper;
use crate::pair_group_queries::{OPairGroupQryTrait, PairAverageDistanceTrait, PairGroupQryArgsCategory, PairGroupQryOutputCategoryParryDistance, PairGroupQryOutputCategoryTrait, PairSkipsTrait, ParryPairSelector};
use crate::pair_queries::{ParryDisMode, ParryShapeRep};
use crate::shapes::{ShapeCategoryOParryShape, ShapeCategoryTrait};
use serde_with::serde_as;
use ad_trait::SerdeAD;

#[derive(Serialize, Deserialize)]
pub struct ProximaGenericContainer<T: AD, P: O3DPose<T>> {
    #[serde(deserialize_with = "AHashMapWrapper::<(u64, u64), ProximaGenericBlock::<T, P>>::deserialize")]
    blocks: AHashMapWrapper<(u64, u64), ProximaGenericBlock<T, P>>
}

#[derive(Serialize, Deserialize)]
pub struct ProximaGenericBlock<T: AD, P: O3DPose<T>> {
    #[serde(deserialize_with = "RwLock::<P>::deserialize")]
    disp_at_j: RwLock<P>,
    #[serde(deserialize_with = "RwLock::<[<P::RotationType as O3DRotation<T>>::Native3DVecType; 2]>::deserialize")]
    closest_points_at_j: RwLock<[<P::RotationType as O3DRotation<T>>::Native3DVecType; 2]>
}

pub struct ParryProximaDistanceQry;
impl OPairGroupQryTrait for ParryProximaDistanceQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = ParryPairSelector;
    type ArgsCategory = PairGroupQryArgsCategoryParryProximaDistance;
    type OutputCategory = PairGroupQryOutputCategoryParryDistance;

    fn query<'a, T: AD, P: O3DPose<T>, S: PairSkipsTrait, A: PairAverageDistanceTrait<T>>(_shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _poses_a: &Vec<P>, _poses_b: &Vec<P>, _pair_selector: &Self::SelectorType, _pair_skips: &S, _pair_average_distances: &A, _args: &<Self::ArgsCategory as PairGroupQryArgsCategory>::Args<'a, T>) -> <Self::OutputCategory as PairGroupQryOutputCategoryTrait>::Output<T, P> {




        todo!()
    }
}

pub struct PairGroupQryArgsCategoryParryProximaDistance;
impl PairGroupQryArgsCategory for PairGroupQryArgsCategoryParryProximaDistance {
    type Args<'a, T: AD> = PairGroupQryArgsParryProximaDistance<T>;
    type QueryType = ParryProximaDistanceQry;
}

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct PairGroupQryArgsParryProximaDistance<T: AD> {
    #[serde(deserialize_with = "ProximaGenericContainer::<T, Isometry3::<T>>::deserialize")]
    proxima_container: ProximaGenericContainer<T, Isometry3<T>>,
    parry_shape_rep: ParryShapeRep,
    parry_dis_mode: ParryDisMode,
    use_average_distance: bool,
    for_filter: bool,
    #[serde_as(as = "SerdeAD<T>")]
    termination_distance_threshold: T
}