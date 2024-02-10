use std::any::Any;
use std::borrow::Cow;
use std::collections::HashMap;
use std::time::{Duration, Instant};
use ad_trait::{AD};
use ahash::AHashMap;
use as_any::AsAny;
use serde::{Deserialize, Serialize};
use serde_with::serde_as;
use optima_3d_spatial::optima_3d_pose::{O3DPose};
use optima_linalg::OVec;
use optima_universal_hashmap::AHashMapWrapper;
use serde_with::*;
use crate::pair_queries::{OPairQryTrait, ParryContactOutput, ParryContactQry, ParryDisMode, ParryDistanceOutput, ParryDistanceQry, ParryIntersectOutput, ParryIntersectQry, ParryOutputAuxData, ParryQryShapeType, ParryShapeRep};
use crate::shape_queries::{ContactOutputTrait, DistanceOutputTrait, IntersectOutputTrait};
use crate::shapes::{OParryShape, ShapeCategoryOParryShape, ShapeCategoryTrait};
use ad_trait::SerdeAD;
use serde::de::DeserializeOwned;
use optima_file::traits::{FromJsonString, ToJsonString};
use as_any::Downcast;

////////////////////////////////////////////////////////////////////////////////////////////////////
pub trait OPairGroupQryTrait {
    type ShapeCategory : ShapeCategoryTrait;
    type SelectorType : AsAny;
    type ArgsCategory : OPairGroupQryArgsCategoryTrait;
    type OutputCategory : OPairGroupQryOutputCategoryTrait;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, freeze: bool, args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P>;
}
impl OPairGroupQryTrait for () {
    type ShapeCategory = ();
    type SelectorType = ();
    type ArgsCategory = ();
    type OutputCategory = ();

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(_shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _poses_a: &Vec<P>, _poses_b: &Vec<P>, _pair_selector: &Self::SelectorType, _pair_skips: &S, _pair_average_distances: &A, _freeze: bool, _args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        ()
    }
}
pub type OwnedEmptyOPairGroupQry<T> = OwnedPairGroupQry<T, ()>;

pub trait OPairGroupQryArgsCategoryTrait {
    type Args<T: AD> : Any + Serialize + DeserializeOwned;
    type QueryType : OPairGroupQryTrait;
}
impl OPairGroupQryArgsCategoryTrait for () {
    type Args<T: AD> = ();
    type QueryType = ();
}

pub trait OPairGroupQryOutputCategoryTrait {
    type Output<T: AD, P: O3DPose<T>> : Any;
}
impl OPairGroupQryOutputCategoryTrait for () {
    type Output<T: AD, P: O3DPose<T>> = ();
}

impl<C: OPairGroupQryOutputCategoryTrait> OPairGroupQryOutputCategoryTrait for Box<C> {
    type Output<T: AD, P: O3DPose<T>> = C::Output<T, P>;
}

#[derive(Serialize, Deserialize)]
pub struct OwnedPairGroupQry<T: AD, Q: OPairGroupQryTrait> {
    #[serde(deserialize_with = "<Q::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args::<T>::deserialize")]
    args: <Q::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>
}
impl<T: AD, Q: OPairGroupQryTrait> OwnedPairGroupQry<T, Q> {
    pub fn new(args: <Q::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> Self {
        Self { args }
    }
    pub fn query<S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>, P: O3DPose<T>>(&self, shape_group_a: &Vec<<Q::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Q::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Q::SelectorType, pair_skips: &S, pair_average_distances: &A, freeze: bool) -> <Q::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        Q::query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, pair_skips, pair_average_distances, freeze, &self.args)
    }
    pub fn to_other_ad_type<T1: AD>(&self) -> OwnedPairGroupQry<T1, Q> {
        let json_str = self.to_json_string();
        OwnedPairGroupQry::<T1, Q>::from_json_string(&json_str)
        // let new_args: <Q::ArgsCategory as PairGroupQryArgsCategory>::Args<'a, T1> = <Q::ArgsCategory as PairGroupQryArgsCategory>::ArgsConverterType::convert_to_other_ad_type::<T, T1>(&self.args);
        // OwnedPairGroupQry::<'a, T1, Q>::new(new_args)
    }
    #[inline(always)]
    pub fn try_downcast_args<Q1: OPairGroupQryArgsCategoryTrait>(&self) -> Option<&Q1::Args<T>> {
        self.args.downcast_ref::<Q1::Args<T>>()
    }
}
impl<'a, T: AD, Q: OPairGroupQryTrait> Clone for OwnedPairGroupQry<T, Q> {
    fn clone(&self) -> Self {
        self.to_other_ad_type::<T>()
    }
}

pub trait OPairSkipsTrait {
    fn skip(&self, shape_a_id: u64, shape_b_id: u64) -> bool;
    #[inline(always)]
    fn skip_reasons(&self, shape_a_id: u64, shape_b_id: u64) -> Option<Cow<Vec<OSkipReason>>> {
        let skip = self.skip(shape_a_id, shape_b_id);
        return if skip { Some(Cow::Owned(vec![])) } else { None }
    }
}
impl OPairSkipsTrait for () {
    fn skip(&self, _shape_a_id: u64, _shape_b_id: u64) -> bool {
        false
    }
}
impl OPairSkipsTrait for HashMap<(u64, u64), ()> {
    fn skip(&self, shape_a_id: u64, shape_b_id: u64) -> bool {
        let res = self.get(&(shape_a_id, shape_b_id));
        return match res {
            None => { false }
            Some(_) => { true }
        }
    }
}
impl OPairSkipsTrait for AHashMap<(u64, u64), ()> {
    #[inline(always)]
    fn skip(&self, shape_a_id: u64, shape_b_id: u64) -> bool {
        let res = self.get(&(shape_a_id, shape_b_id));
        return match res {
            None => { false }
            Some(_) => { true }
        }
    }
}
impl OPairSkipsTrait for AHashMapWrapper<(u64, u64), ()> {
    #[inline(always)]
    fn skip(&self, shape_a_id: u64, shape_b_id: u64) -> bool {
        let res = self.hashmap.get(&(shape_a_id, shape_b_id));
        return match res {
            None => { false }
            Some(_) => { true }
        }
    }
}
impl OPairSkipsTrait for AHashMapWrapper<(u64, u64), Vec<OSkipReason>> {
    #[inline(always)]
    fn skip(&self, shape_a_id: u64, shape_b_id: u64) -> bool {
        self.hashmap.contains_key(&(shape_a_id, shape_b_id))
    }
    #[inline(always)]
    fn skip_reasons(&self, shape_a_id: u64, shape_b_id: u64) -> Option<Cow<Vec<OSkipReason>>> {
        let reasons = self.hashmap.get(&(shape_a_id, shape_b_id));
        return match reasons {
            None => { None }
            Some(reasons) => { Some(Cow::Borrowed(reasons)) }
        }
    }
}
pub trait AHashMapWrapperSkipsWithReasonsTrait {
    fn clear_skip_reason_type(&mut self, reason: OSkipReason);
    fn add_skip_reason(&mut self, shape_a_id: u64, shape_b_id: u64, reason: OSkipReason);
}
impl AHashMapWrapperSkipsWithReasonsTrait for AHashMapWrapper<(u64, u64), Vec<OSkipReason>> {
    fn clear_skip_reason_type(&mut self, reason: OSkipReason) {
        self.hashmap.iter_mut().for_each(|x| {
            let idx = x.1.iter().position(|y| *y == reason);
            match idx {
                None => {}
                Some(idx) => { x.1.remove(idx); }
            }
        });

        self.hashmap = self.hashmap.iter().filter(|x| !x.1.is_empty() ).map(|(x,y)|(*x, y.clone())).collect();
    }

    fn add_skip_reason(&mut self, shape_a_id: u64, shape_b_id: u64, reason: OSkipReason) {
        let res_mut = self.hashmap.get_mut(&(shape_a_id, shape_b_id));
        match res_mut {
            None => {
                self.hashmap.insert((shape_a_id, shape_b_id), vec![reason]);
            }
            Some(res_mut) => {
                if !res_mut.contains(&reason) {
                    res_mut.push(reason);
                }
            }
        }
    }
}
#[derive(Copy, Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum OSkipReason {
    AlwaysInCollision, NeverInCollision, FromNonCollisionExample,
    CloseProximityWrtAverageExample
}

pub trait OPairAverageDistanceTrait<T: AD> {
    fn average_distance(&self, shape_a_id: u64, shape_b_id: u64) -> T;
}
impl<T: AD> OPairAverageDistanceTrait<T> for AHashMapWrapper<(u64, u64), T> {
    #[inline(always)]
    fn average_distance(&self, shape_a_id: u64, shape_b_id: u64) -> T {
        *self.hashmap.get(&(shape_a_id, shape_b_id)).expect(&format!("error {:?}", (shape_a_id, shape_b_id)))
    }
}
impl<T: AD> OPairAverageDistanceTrait<T> for () {
    #[inline(always)]
    fn average_distance(&self, _shape_a_id: u64, _shape_b_id: u64) -> T {
        T::one()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum OParryPairSelector {
    AllPairs,
    HalfPairs,
    AllPairsSubcomponents,
    HalfPairsSubcomponents,
    PairsByIdxs(Vec<OParryPairIdxs>)
}
impl OParryPairSelector {
    pub fn len(&self) -> usize {
        match self {
            OParryPairSelector::PairsByIdxs(v) => { v.len() }
            _ => { usize::MAX }
        }
    }
}

pub struct OParryPairGroupOutputWrapper<O> {
    data: O,
    pair_ids: (u64, u64),
    pair_idxs: OParryPairIdxs
}
impl<O> OParryPairGroupOutputWrapper<O> {
    #[inline(always)]
    pub fn data(&self) -> &O {
        &self.data
    }
    #[inline(always)]
    pub fn pair_idxs(&self) -> &OParryPairIdxs {
        &self.pair_idxs
    }
    #[inline(always)]
    pub fn pair_ids(&self) -> (u64, u64) {
        self.pair_ids
    }
}
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum OParryPairIdxs {
    Shapes(usize, usize),
    ShapeSubcomponents((usize, usize), (usize, usize))
}
impl OParryPairIdxs {
    pub fn to_parry_shape_idxs(&self) -> (OParryShapeIdx, OParryShapeIdx) {
        match self {
            OParryPairIdxs::Shapes(x, y) => { (OParryShapeIdx::Shape(*x), OParryShapeIdx::Shape(*y)) }
            OParryPairIdxs::ShapeSubcomponents(x, y) => { (OParryShapeIdx::ShapeSubcomponent(*x), OParryShapeIdx::ShapeSubcomponent(*y)) }
        }
    }
}

#[derive(Debug, Clone)]
pub enum OParryShapeIdx {
    Shape(usize),
    ShapeSubcomponent((usize, usize))
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// INTERSECT //
pub struct OParryIntersectGroupQry;
impl OPairGroupQryTrait for OParryIntersectGroupQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = OParryIntersectGroupArgsCategory;
    type OutputCategory = OParryIntersectGroupOutputCategory;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, _pair_average_distances: &A, _freeze: bool, args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep1: &ParryShapeRep, parry_shape_rep2: &ParryShapeRep| -> ParryIntersectOutput {
            ParryIntersectQry::query(shape_a, shape_b, pose_a, pose_b, &(parry_qry_shape_type.clone(), parry_shape_rep1.clone(), parry_shape_rep2.clone()))
        };

        let termination = if args.terminate_on_first_intersection {
            |o: &ParryIntersectOutput| {
                if o.intersect() { return true } else { false }
            }
        } else {
            |_o: &ParryIntersectOutput| {
                false
            }
        };

        let (mut outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep1, &args.parry_shape_rep2, pair_skips, args.for_filter, f, termination);

        outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        Box::new(OParryIntersectGroupOutput {
            intersect: if outputs.len() == 0 { false } else { outputs[0].data.intersect() },
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        })
    }
}
pub type OwnedParryIntersectGroupQry<T> = OwnedPairGroupQry<T, OParryIntersectGroupQry>;

pub struct OPairGroupQryCategoryParryIntersect;

#[derive(Clone, Serialize, Deserialize)]
pub struct OParryIntersectGroupArgs {
    parry_shape_rep1: ParryShapeRep,
    parry_shape_rep2: ParryShapeRep,
    terminate_on_first_intersection: bool,
    for_filter: bool
}
impl OParryIntersectGroupArgs {
    pub fn new(parry_shape_rep1: ParryShapeRep, parry_shape_rep2: ParryShapeRep, terminate_on_first_intersection: bool, for_filter: bool) -> Self {
        Self { parry_shape_rep1, parry_shape_rep2, terminate_on_first_intersection, for_filter }
    }
}

pub struct OParryIntersectGroupArgsCategory;
impl OPairGroupQryArgsCategoryTrait for OParryIntersectGroupArgsCategory {
    type Args<T: AD> = OParryIntersectGroupArgs;
    type QueryType = OParryIntersectGroupQry;
}

/*
pub struct PairGroupQryArgsCategoryParryIntersectConverter;
impl ADConvertableTrait for PairGroupQryArgsCategoryParryIntersectConverter {
    type ConvertableType<T: AD> = OParryIntersectGroupArgs;

    fn convert_to_other_ad_type<T1: AD, T2: AD>(input: &Self::ConvertableType<T1>) -> Self::ConvertableType<T2> {
        input.clone()
    }
}
*/

pub struct OParryIntersectGroupOutput {
    intersect: bool,
    outputs: Vec<OParryPairGroupOutputWrapper<ParryIntersectOutput>>,
    aux_data: ParryOutputAuxData
}
impl OParryIntersectGroupOutput {
    pub fn intersect(&self) -> bool {
        self.intersect
    }
    pub fn outputs(&self) -> &Vec<OParryPairGroupOutputWrapper<ParryIntersectOutput>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

pub struct OParryIntersectGroupOutputCategory;
impl OPairGroupQryOutputCategoryTrait for OParryIntersectGroupOutputCategory {
    type Output<T: AD, P: O3DPose<T>> = Box<OParryIntersectGroupOutput>;
}

pub struct EmptyParryPairGroupIntersectQry;
impl OPairGroupQryTrait for EmptyParryPairGroupIntersectQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = ();
    type OutputCategory = OParryIntersectGroupOutputCategory;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(_shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _poses_a: &Vec<P>, _poses_b: &Vec<P>, _pair_selector: &Self::SelectorType, _pair_skips: &S, _pair_average_distances: &A, _freeze: bool, _args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        Box::new(OParryIntersectGroupOutput {
            intersect: false,
            outputs: vec![],
            aux_data: ParryOutputAuxData { num_queries: 0, duration: Default::default() },
        })
    }
}
pub type OwnedEmptyParryPairGroupIntersectQry<T> = OwnedPairGroupQry<T, EmptyParryPairGroupIntersectQry>;

////////////////////////////////////////////////////////////////////////////////////////////////////

// DISTANCE //

pub struct OParryDistanceGroupQry;
impl OPairGroupQryTrait for OParryDistanceGroupQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = OParryDistanceGroupArgsCategory;
    type OutputCategory = OParryDistanceGroupOutputCategory;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, _freeze: bool, args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep1: &ParryShapeRep, parry_shape_rep2: &ParryShapeRep| -> ParryDistanceOutput<T> {
            let a = get_average_distance_option_from_shape_pair(args.use_average_distance, shape_a, shape_b, parry_qry_shape_type, parry_shape_rep1, parry_shape_rep2, args.for_filter, pair_average_distances);
            ParryDistanceQry::query(shape_a, shape_b, pose_a, pose_b, &(args.parry_dis_mode.clone(), parry_qry_shape_type.clone(), parry_shape_rep1.clone(), parry_shape_rep2.clone(), a))
        };

        let termination = |o: &ParryDistanceOutput<T>| {
            return o.distance() <= args.termination_distance_threshold
        };

        let (mut outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep1, &args.parry_shape_rep2, pair_skips, args.for_filter, f, termination);

        if args.sort_outputs {
            outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());
        }

        Box::new(OParryDistanceGroupOutput {
            min_dis_wrt_average: if outputs.len() == 0 { T::constant(100_000_000.0) } else { outputs[0].data.distance_wrt_average },
            min_raw_dis: if outputs.len() == 0 { T::constant(100_000_000.0) } else { outputs[0].data.raw_distance },
            sorted: args.sort_outputs,
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        })
    }
}
pub type OwnedParryDistanceGroupQry<T> = OwnedPairGroupQry<T, OParryDistanceGroupQry>;

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct OParryDistanceGroupArgs<T: AD> {
    parry_shape_rep1: ParryShapeRep,
    parry_shape_rep2: ParryShapeRep,
    parry_dis_mode: ParryDisMode,
    use_average_distance: bool,
    for_filter: bool,
    #[serde_as(as = "SerdeAD<T>")]
    termination_distance_threshold: T,
    sort_outputs: bool
}
impl<T: AD> OParryDistanceGroupArgs<T> {
    pub fn new(parry_shape_rep1: ParryShapeRep, parry_shape_rep2: ParryShapeRep, parry_dis_mode: ParryDisMode, use_average_distance: bool, for_filter: bool, termination_distance_threshold: T, sort_outputs: bool) -> Self {
        Self { parry_shape_rep1, parry_shape_rep2, parry_dis_mode, use_average_distance, for_filter, termination_distance_threshold, sort_outputs }
    }
}

pub struct OParryDistanceGroupArgsCategory;
impl OPairGroupQryArgsCategoryTrait for OParryDistanceGroupArgsCategory {
    type Args<T: AD> = OParryDistanceGroupArgs<T>;
    type QueryType = OParryDistanceGroupQry;
}

/*
pub struct PairGroupQryArgsCategoryParryDistanceConverter;
impl ADConvertableTrait for PairGroupQryArgsCategoryParryDistanceConverter {
    type ConvertableType<T: AD> = OParryDistanceGroupArgs<T>;

    fn convert_to_other_ad_type<T1: AD, T2: AD>(input: &Self::ConvertableType<T1>) -> Self::ConvertableType<T2> {
        let json_str = input.to_json_string();
        Self::ConvertableType::<T2>::from_json_string(&json_str)
    }
}
*/

pub struct OParryDistanceGroupOutput<T: AD> {
    min_dis_wrt_average: T,
    min_raw_dis: T,
    sorted: bool,
    outputs: Vec<OParryPairGroupOutputWrapper<ParryDistanceOutput<T>>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD> OParryDistanceGroupOutput<T> {
    pub fn min_dis_wrt_average(&self) -> &T {
        assert!(self.sorted, "must be sorted in order to get minimum in this way");
        &self.min_dis_wrt_average
    }
    pub fn min_raw_dis(&self) -> &T {
        assert!(self.sorted, "must be sorted in order to get minimum in this way");
        &self.min_raw_dis
    }
    pub fn outputs(&self) -> &Vec<OParryPairGroupOutputWrapper<ParryDistanceOutput<T>>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl<T: AD> ToParryProximityOutputTrait<T> for OParryDistanceGroupOutput<T> {
    fn get_proximity_objective_value(&self, cutoff: T, p_norm: T, loss_function: OProximityLossFunction) -> T {
        let mut values = vec![];

        self.outputs.iter().for_each(|x| {
            let loss = loss_function.loss(x.data.distance_wrt_average, cutoff);
            values.push(loss);
        });

        let out = values.ovec_p_norm(&p_norm);
        out
    }
}

pub struct OParryDistanceGroupOutputCategory;
impl OPairGroupQryOutputCategoryTrait for OParryDistanceGroupOutputCategory {
    type Output<T: AD, P: O3DPose<T>> = Box<OParryDistanceGroupOutput<T>>;
}

pub struct EmptyParryPairGroupDistanceQry;
impl OPairGroupQryTrait for EmptyParryPairGroupDistanceQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = ();
    type OutputCategory = OParryDistanceGroupOutputCategory;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(_shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _poses_a: &Vec<P>, _poses_b: &Vec<P>, _pair_selector: &Self::SelectorType, _pair_skips: &S, _pair_average_distances: &A, _freeze: bool, _args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        Box::new(OParryDistanceGroupOutput {
            min_dis_wrt_average: T::constant(f64::MAX),
            min_raw_dis: T::constant(f64::MAX),
            sorted: true,
            outputs: vec![],
            aux_data: ParryOutputAuxData { num_queries: 0, duration: Default::default() },
        })
    }
}
pub type OwnedEmptyParryPairGroupDistanceQry<T> = OwnedPairGroupQry<T, EmptyParryPairGroupDistanceQry>;

////////////////////////////////////////////////////////////////////////////////////////////////////

// DISTANCE AS PROXIMITY //

pub struct OParryDistanceAsProximityGroupQry;
impl OPairGroupQryTrait for OParryDistanceAsProximityGroupQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = OParryDistanceGroupArgsCategory;
    type OutputCategory = ToParryProximityOutputCategory;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, _freeze: bool, args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        OParryDistanceGroupQry::query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, pair_skips, pair_average_distances, _freeze, args)
    }
}
pub type OwnedParryDistanceAsProximityGroupQry<T> = OwnedPairGroupQry<T, OParryDistanceAsProximityGroupQry>;

////////////////////////////////////////////////////////////////////////////////////////////////////

// CONTACT //

pub struct OParryContactGroupQry;
impl OPairGroupQryTrait for OParryContactGroupQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = OParryContactGroupArgsCategory;
    type OutputCategory = OParryContactGroupOutputCategory;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, _freeze: bool, args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep1: &ParryShapeRep, parry_shape_rep2: &ParryShapeRep| -> ParryContactOutput<T> {
            let a = get_average_distance_option_from_shape_pair(args.use_average_distance, shape_a, shape_b, parry_qry_shape_type, parry_shape_rep1, parry_shape_rep2, args.for_filter, pair_average_distances);
            ParryContactQry::query(shape_a, shape_b, pose_a, pose_b, &(args.contact_threshold.clone(), parry_qry_shape_type.clone(), parry_shape_rep1.clone(), parry_shape_rep2.clone(), a))
        };

        let termination = |o: &ParryContactOutput<T>| {
            let signed_distance = o.signed_distance();
            return match &signed_distance {
                None => { false }
                Some(signed_distance) => { *signed_distance <= args.termination_distance_threshold }
            }
        };

        let (mut outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep1, &args.parry_shape_rep2, pair_skips, args.for_filter, f, termination);

        outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        Box::new(OParryContactGroupOutput {
            min_signed_dis_wrt_average: if outputs.len() == 0 { None } else { outputs[0].data.signed_distance() },
            min_raw_signed_dis: if outputs.len() == 0 { None } else { outputs[0].data.signed_distance() },
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        })
    }
}
pub type OwnedParryContactGroupQry<T> = OwnedPairGroupQry<T, OParryContactGroupQry>;

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct OParryContactGroupArgs<T: AD> {
    parry_shape_rep1: ParryShapeRep,
    parry_shape_rep2: ParryShapeRep,
    #[serde_as(as = "SerdeAD<T>")]
    contact_threshold: T,
    use_average_distance: bool,
    for_filter: bool,
    #[serde_as(as = "SerdeAD<T>")]
    termination_distance_threshold: T
}
impl<T: AD> OParryContactGroupArgs<T> {
    pub fn new(parry_shape_rep1: ParryShapeRep, parry_shape_rep2: ParryShapeRep, contact_threshold: T, use_average_distance: bool, for_filter: bool, termination_distance_threshold: T) -> Self {
        Self { parry_shape_rep1, parry_shape_rep2, contact_threshold, use_average_distance, for_filter, termination_distance_threshold }
    }
}

pub struct OParryContactGroupArgsCategory;
impl OPairGroupQryArgsCategoryTrait for OParryContactGroupArgsCategory {
    type Args<T: AD> = OParryContactGroupArgs<T>;
    type QueryType = OParryContactGroupQry;
}

/*
pub struct PairGroupQryArgsCategoryParryContactConverter;
impl ADConvertableTrait for PairGroupQryArgsCategoryParryContactConverter {
    type ConvertableType<T: AD> = OParryContactGroupArgs<T>;

    fn convert_to_other_ad_type<T1: AD, T2: AD>(input: &Self::ConvertableType<T1>) -> Self::ConvertableType<T2> {
        let json_str = input.to_json_string();
        Self::ConvertableType::<T2>::from_json_string(&json_str)
    }
}
*/

pub struct OParryContactGroupOutput<T: AD> {
    min_signed_dis_wrt_average: Option<T>,
    min_raw_signed_dis: Option<T>,
    outputs: Vec<OParryPairGroupOutputWrapper<ParryContactOutput<T>>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD> OParryContactGroupOutput<T> {
    pub fn min_signed_dis_wrt_average(&self) -> &Option<T> {
        &self.min_signed_dis_wrt_average
    }
    pub fn min_raw_signed_dis(&self) -> &Option<T> {
        &self.min_raw_signed_dis
    }
    pub fn outputs(&self) -> &Vec<OParryPairGroupOutputWrapper<ParryContactOutput<T>>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

/*
impl<T: AD> ToParryProximityOutputTrait<T> for ParryContactGroupOutput<T> {
    fn compute_proximity_objective_value<LF: ProximityLossFunctionTrait<T>>(&self, cutoff: T, p_norm: T, loss_function: LF) -> T {
        let mut values = vec![];

        self.outputs.iter().for_each(|x| {
            match x.data.distance_wrt_average {
                None => {}
                Some(d) => {
                    let loss = loss_function.loss(d, cutoff);
                    values.push(loss);
                }
            }
        });

        values.ovec_p_norm(&p_norm)
    }
}
*/

pub struct OParryContactGroupOutputCategory;
impl OPairGroupQryOutputCategoryTrait for OParryContactGroupOutputCategory {
    type Output<T: AD, P: O3DPose<T>> = Box<OParryContactGroupOutput<T>>;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
pub struct ParryDistanceLowerBoundGroupQry;
impl OPairGroupQryTrait for ParryDistanceLowerBoundGroupQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = ParryPairSelector;
    type ArgsCategory = PairGroupQryArgsCategoryParryDistanceLowerBound;
    type OutputCategory = PairGroupQryOutputCategoryParryDistanceLowerBound;

    fn query<'a, T: AD, P: O3DPose<T>, S: PairSkipsTrait, A: PairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, _proxima_container: &PC, args: &<Self::ArgsCategory as PairGroupQryArgsCategory>::Args<'a, T>) -> <Self::OutputCategory as PairGroupQryOutputCategoryTrait>::Output<T, P> {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryDistanceLowerBoundOutput<T> {
            // ParryContactQry::query(shape_a, shape_b, pose_a, pose_b, &(args.contact_threshold.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone()))
            let a = get_average_distance_option_from_shape_pair(args.use_average_distance, shape_a, shape_b, parry_qry_shape_type, parry_shape_rep, args.for_filter, pair_average_distances);
            ParryDistanceLowerBoundQry::query(shape_a, shape_b, pose_a, pose_b, &(args.parry_dis_mode.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone(), a))
        };

        let termination = |_o: &ParryDistanceLowerBoundOutput<T>| {
            false
        };

        let (outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, args.for_filter, f, termination);

        // outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        ParryDistanceLowerBoundGroupOutput {
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}
pub type OwnedParryDistanceLowerBoundGroupQry<'a, T> = OwnedPairGroupQry<'a, T, ParryDistanceLowerBoundGroupQry>;

#[derive(Serialize, Deserialize)]
pub struct ParryDistanceLowerBoundGroupArgs {
    parry_dis_mode: ParryDisMode,
    parry_shape_rep: ParryShapeRep,
    use_average_distance: bool,
    for_filter: bool
}
impl ParryDistanceLowerBoundGroupArgs {
    pub fn new(parry_dis_mode: ParryDisMode, parry_shape_rep: ParryShapeRep, use_average_distance: bool, for_filter: bool) -> Self {
        Self { parry_dis_mode, parry_shape_rep, use_average_distance, for_filter }
    }
}

pub struct PairGroupQryArgsCategoryParryDistanceLowerBound;
impl PairGroupQryArgsCategory for PairGroupQryArgsCategoryParryDistanceLowerBound {
    type Args<T: AD> = ParryDistanceLowerBoundGroupArgs;
    type QueryType = ParryDistanceLowerBoundGroupQry;
}

pub struct ParryDistanceLowerBoundGroupOutput<T: AD> {
    outputs: Vec<ParryPairGroupOutputWrapper<ParryDistanceLowerBoundOutput<T>>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceLowerBoundGroupOutput<T> {
    pub fn outputs(&self) -> &Vec<ParryPairGroupOutputWrapper<ParryDistanceLowerBoundOutput<T>>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

pub struct PairGroupQryOutputCategoryParryDistanceLowerBound;
impl PairGroupQryOutputCategoryTrait for PairGroupQryOutputCategoryParryDistanceLowerBound {
    type Output<T: AD, P: O3DPose<T>> = ParryDistanceLowerBoundGroupOutput<T>;
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
pub struct ParryDistanceUpperBoundGroupQry;
impl OPairGroupQryTrait for ParryDistanceUpperBoundGroupQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = ParryPairSelector;
    type ArgsCategory = PairGroupQryArgsCategoryParryDistanceUpperBound;
    type OutputCategory = PairGroupQryOutputCategoryParryDistanceUpperBound;

    fn query<'a, T: AD, P: O3DPose<T>, S: PairSkipsTrait, A: PairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, _proxima_container: &PC, args: &<Self::ArgsCategory as PairGroupQryArgsCategory>::Args<'a, T>) -> <Self::OutputCategory as PairGroupQryOutputCategoryTrait>::Output<T, P> {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryDistanceUpperBoundOutput<T> {
            // ParryContactQry::query(shape_a, shape_b, pose_a, pose_b, &(args.contact_threshold.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone()))
            let a = get_average_distance_option_from_shape_pair(args.use_average_distance, shape_a, shape_b, parry_qry_shape_type, parry_shape_rep, args.for_filter, pair_average_distances);
            ParryDistanceUpperBoundQry::query(shape_a, shape_b, pose_a, pose_b, &(args.parry_dis_mode.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone(), a))
        };

        let termination = |_o: &ParryDistanceUpperBoundOutput<T>| {
            false
        };

        let (outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, args.for_filter, f, termination);

        // outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        ParryDistanceUpperBoundGroupOutput {
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}
pub type OwnedParryDistanceUpperBoundGroupQry<'a, T> = OwnedPairGroupQry<'a, T, ParryDistanceUpperBoundGroupQry>;

#[derive(Serialize, Deserialize)]
pub struct ParryDistanceUpperBoundGroupArgs {
    parry_dis_mode: ParryDisMode,
    parry_shape_rep: ParryShapeRep,
    use_average_distance: bool,
    for_filter: bool
}
impl ParryDistanceUpperBoundGroupArgs {
    pub fn new(parry_dis_mode: ParryDisMode, parry_shape_rep: ParryShapeRep, use_average_distance: bool, for_filter: bool) -> Self {
        Self { parry_dis_mode, parry_shape_rep, use_average_distance, for_filter }
    }
}

pub struct PairGroupQryArgsCategoryParryDistanceUpperBound;
impl PairGroupQryArgsCategory for PairGroupQryArgsCategoryParryDistanceUpperBound {
    type Args<T: AD> = ParryDistanceUpperBoundGroupArgs;
    type QueryType = ParryDistanceUpperBoundGroupQry;
}

pub struct ParryDistanceUpperBoundGroupOutput<T: AD> {
    outputs: Vec<ParryPairGroupOutputWrapper<ParryDistanceUpperBoundOutput<T>>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceUpperBoundGroupOutput<T> {
    pub fn outputs(&self) -> &Vec<ParryPairGroupOutputWrapper<ParryDistanceUpperBoundOutput<T>>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

pub struct PairGroupQryOutputCategoryParryDistanceUpperBound;
impl PairGroupQryOutputCategoryTrait for PairGroupQryOutputCategoryParryDistanceUpperBound {
    type Output<T: AD, P: O3DPose<T>> = ParryDistanceUpperBoundGroupOutput<T>;
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
pub struct ParryDistanceBoundsGroupQry;
impl OPairGroupQryTrait for ParryDistanceBoundsGroupQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = ParryPairSelector;
    type ArgsCategory = PairGroupQryArgsCategoryParryDistanceBounds;
    type OutputCategory = PairGroupQryOutputCategoryParryDistanceBounds;

    fn query<'a, T: AD, P: O3DPose<T>, S: PairSkipsTrait, A: PairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, _proxima_container: &PC, args: &<Self::ArgsCategory as PairGroupQryArgsCategory>::Args<'a, T>) -> <Self::OutputCategory as PairGroupQryOutputCategoryTrait>::Output<T, P> {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryDistanceBoundsOutput<T> {
            // ParryContactQry::query(shape_a, shape_b, pose_a, pose_b, &(args.contact_threshold.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone()))
            let a = get_average_distance_option_from_shape_pair(args.use_average_distance, shape_a, shape_b, parry_qry_shape_type, parry_shape_rep, args.for_filter, pair_average_distances);
            ParryDistanceBoundsQry::query(shape_a, shape_b, pose_a, pose_b, &(args.parry_dis_mode.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone(), a))
        };

        let termination = |_o: &ParryDistanceBoundsOutput<T>| {
            false
        };

        let (outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, args.for_filter, f, termination);

        // outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        ParryDistanceBoundsGroupOutput {
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}
pub type OwnedParryDistanceBoundsGroupQry<'a, T> = OwnedPairGroupQry<'a, T, ParryDistanceBoundsGroupQry>;

#[derive(Serialize, Deserialize)]
pub struct ParryDistanceBoundsGroupArgs {
    parry_dis_mode: ParryDisMode,
    parry_shape_rep: ParryShapeRep,
    use_average_distance: bool,
    for_filter: bool
}
impl ParryDistanceBoundsGroupArgs {
    pub fn new(parry_dis_mode: ParryDisMode, parry_shape_rep: ParryShapeRep, use_average_distance: bool, for_filter: bool) -> Self {
        Self { parry_dis_mode, parry_shape_rep, use_average_distance, for_filter }
    }
}

pub struct PairGroupQryArgsCategoryParryDistanceBounds;
impl PairGroupQryArgsCategory for PairGroupQryArgsCategoryParryDistanceBounds {
    type Args<T: AD> = ParryDistanceBoundsGroupArgs;
    type QueryType = ParryDistanceBoundsGroupQry;
}

pub struct ParryDistanceBoundsGroupOutput<T: AD> {
    outputs: Vec<ParryPairGroupOutputWrapper<ParryDistanceBoundsOutput<T>>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceBoundsGroupOutput<T> {
    pub fn outputs(&self) -> &Vec<ParryPairGroupOutputWrapper<ParryDistanceBoundsOutput<T>>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

pub struct PairGroupQryOutputCategoryParryDistanceBounds;
impl PairGroupQryOutputCategoryTrait for PairGroupQryOutputCategoryParryDistanceBounds {
    type Output<T: AD, P: O3DPose<T>> = ParryDistanceBoundsGroupOutput<T>;
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone)]
pub struct OParryFilterOutput {
    selector: OParryPairSelector,
    duration: Duration,
    aux_datas: Vec<ParryOutputAuxData>
}
impl OParryFilterOutput {
    pub fn selector(&self) -> &OParryPairSelector {
        &self.selector
    }
    pub fn duration(&self) -> Duration {
        self.duration
    }
    pub fn aux_datas(&self) -> &Vec<ParryOutputAuxData> {
        &self.aux_datas
    }
}
impl AsParryFilterOutputTrait for OParryFilterOutput {
    #[inline(always)]
    fn as_parry_filter_output(&self) -> &OParryFilterOutput {
        self
    }
}

pub struct OParryFilterOutputCategory;
impl OPairGroupQryOutputCategoryTrait for OParryFilterOutputCategory {
    type Output<T: AD, P: O3DPose<T>> = OParryFilterOutput;
}

pub trait AsParryFilterOutputTrait {
    fn as_parry_filter_output(&self) -> &OParryFilterOutput;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// DISTANCE FILTER //

pub struct OParryDistanceGroupFilter;
impl OPairGroupQryTrait for OParryDistanceGroupFilter {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = OParryDistanceGroupFilterArgsCategory;
    type OutputCategory = OParryFilterOutputCategory;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, _freeze: bool, args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        let qry = OwnedPairGroupQry::<T, OParryDistanceGroupQry>::new(OParryDistanceGroupArgs::new(args.parry_shape_rep1.clone(), args.parry_shape_rep2.clone(), args.parry_dis_mode.clone(), args.use_average_distance, true, T::constant(f64::MIN), false));
        let f = |output: &Box<OParryDistanceGroupOutput<T>> | -> Vec<OParryPairIdxs> {
            let mut a = vec![];
            output.outputs.iter().for_each(|x| {
                if x.data.distance() < args.distance_threshold {
                    a.push(x.pair_idxs.clone());
                }
            });
            // println!("{:?}, {:?}", output.outputs.len(), a.len());
            a
        };

        parry_generic_pair_group_filter(qry, shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, pair_skips, pair_average_distances, f)
    }
}
pub type OwnedParryDistanceGroupFilter<T> = OwnedPairGroupQry<T, OParryDistanceGroupFilter>;

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct OParryDistanceGroupFilterArgs<T: AD> {
    parry_shape_rep1: ParryShapeRep,
    parry_shape_rep2: ParryShapeRep,
    parry_dis_mode: ParryDisMode,
    use_average_distance: bool,
    #[serde_as(as = "SerdeAD<T>")]
    distance_threshold: T
}
impl<T: AD> OParryDistanceGroupFilterArgs<T> {
    pub fn new(parry_shape_rep1: ParryShapeRep, parry_shape_rep2: ParryShapeRep, parry_dis_mode: ParryDisMode, use_average_distance: bool, distance_threshold: T) -> Self {
        Self { parry_shape_rep1, parry_shape_rep2, parry_dis_mode, use_average_distance, distance_threshold }
    }
}

pub struct OParryDistanceGroupFilterArgsCategory;
impl OPairGroupQryArgsCategoryTrait for OParryDistanceGroupFilterArgsCategory {
    type Args<T: AD> = OParryDistanceGroupFilterArgs<T>;
    type QueryType = OParryDistanceGroupFilter;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// INTERSECT FILTER //

pub struct OParryIntersectGroupFilter;
impl OPairGroupQryTrait for OParryIntersectGroupFilter {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = OParryIntersectGroupFilterArgsCategory;
    type OutputCategory = OParryFilterOutputCategory;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, _freeze: bool, args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        let qry = OwnedPairGroupQry::<T, OParryIntersectGroupQry>::new(OParryIntersectGroupArgs::new(args.parry_shape_rep1.clone(), args.parry_shape_rep2.clone(), false, true));
        let f = |output: &Box<OParryIntersectGroupOutput> | -> Vec<OParryPairIdxs> {
            let mut a = vec![];
            output.outputs.iter().for_each(|x| {
                if x.data.intersect {
                    a.push(x.pair_idxs.clone());
                }
            });
            a
        };

        parry_generic_pair_group_filter(qry, shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, pair_skips, pair_average_distances, f)
    }
}
pub type OwnedParryIntersectGroupFilter<T> = OwnedPairGroupQry<T, OParryIntersectGroupFilter>;

#[derive(Clone, Serialize, Deserialize)]
pub struct OParryIntersectGroupFilterArgs {
    parry_shape_rep1: ParryShapeRep,
    parry_shape_rep2: ParryShapeRep
}
impl OParryIntersectGroupFilterArgs {
    pub fn new(parry_shape_rep1: ParryShapeRep, parry_shape_rep2: ParryShapeRep) -> Self {
        Self { parry_shape_rep1, parry_shape_rep2 }
    }
}

pub struct OParryIntersectGroupFilterArgsCategory;
impl OPairGroupQryArgsCategoryTrait for OParryIntersectGroupFilterArgsCategory {
    type Args<T: AD> = OParryIntersectGroupFilterArgs;
    type QueryType = OParryIntersectGroupFilter;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// TO SUBCOMPONENT FILTER //

pub struct OParryToSubcomponentFilter;
impl OPairGroupQryTrait for OParryToSubcomponentFilter {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = ();
    type OutputCategory = OParryFilterOutputCategory;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, _freeze: bool, _args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        let start = Instant::now();
        let mut subcomponent_idxs = vec![];
        match pair_selector {
            OParryPairSelector::PairsByIdxs(pair_idxs) => {
                pair_idxs.iter().for_each(|x| {
                    match x {
                        OParryPairIdxs::Shapes(x, y) => {
                            let shape_a = &shape_group_a[*x];
                            let shape_b = &shape_group_b[*y];

                            let num_subcomponents_a = shape_a.convex_subcomponents.len();
                            let num_subcomponents_b = shape_b.convex_subcomponents.len();

                            for i in 0..num_subcomponents_a {
                                for j in 0..num_subcomponents_b {
                                    subcomponent_idxs.push(OParryPairIdxs::ShapeSubcomponents((*x, i), (*y, j)));
                                }
                            }
                        }
                        OParryPairIdxs::ShapeSubcomponents(x, y) => {
                            subcomponent_idxs.push(OParryPairIdxs::ShapeSubcomponents(x.clone(), y.clone() ))
                        }
                    }
                });
            }
            OParryPairSelector::AllPairs => {
                for x in 0..shape_group_a.len() {
                    for y in 0..shape_group_b.len() {
                        let shape_a = &shape_group_a[x];
                        let shape_b = &shape_group_b[y];

                        let num_subcomponents_a = shape_a.convex_subcomponents.len();
                        let num_subcomponents_b = shape_b.convex_subcomponents.len();

                        for i in 0..num_subcomponents_a {
                            for j in 0..num_subcomponents_b {
                                subcomponent_idxs.push(OParryPairIdxs::ShapeSubcomponents((x, i), (y, j)));
                            }
                        }
                    }
                }
            }
            OParryPairSelector::HalfPairs => {
                for x in 0..shape_group_a.len() {
                    for y in 0..shape_group_b.len() {
                        if x < y {
                            let shape_a = &shape_group_a[x];
                            let shape_b = &shape_group_b[y];

                            let num_subcomponents_a = shape_a.convex_subcomponents.len();
                            let num_subcomponents_b = shape_b.convex_subcomponents.len();

                            for i in 0..num_subcomponents_a {
                                for j in 0..num_subcomponents_b {
                                    subcomponent_idxs.push(OParryPairIdxs::ShapeSubcomponents((x, i), (y, j)));
                                }
                            }
                        }
                    }
                }
            }
            OParryPairSelector::AllPairsSubcomponents => {
                return Self::query(shape_group_a, shape_group_b, poses_a, poses_b, &OParryPairSelector::AllPairs, pair_skips, pair_average_distances, false, &());
            }
            OParryPairSelector::HalfPairsSubcomponents => {
                return Self::query(shape_group_a, shape_group_b, poses_a, poses_b, &OParryPairSelector::HalfPairs, pair_skips, pair_average_distances, false, &());
            }
        }

        let selector = OParryPairSelector::PairsByIdxs(subcomponent_idxs);

        OParryFilterOutput {
            selector,
            duration: start.elapsed(),
            aux_datas: vec![ParryOutputAuxData { num_queries: 0, duration: start.elapsed() }],
        }
    }
}
pub type OwnedParryToSubcomponentFilter<T> = OwnedPairGroupQry<T, OParryToSubcomponentFilter>;

////////////////////////////////////////////////////////////////////////////////////////////////////

// INTERSECT SEQUENCE FILTER //

pub struct OParryIntersectGroupSequenceFilter;
impl OPairGroupQryTrait for OParryIntersectGroupSequenceFilter {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = OParryIntersectGroupSequenceFilterArgsCategory;
    type OutputCategory = OParryFilterOutputCategory;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, _freeze: bool, args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        let start = Instant::now();

        let mut curr = pair_selector.clone();
        let mut aux_datas = vec![];

        args.shape_rep_seq.iter().for_each(|x| {
            // let f = ParryIntersectGroupFilter2::new(x.clone());
            // let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances);
            let res = OParryIntersectGroupFilter::query(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances, false, &OParryIntersectGroupFilterArgs::new(x.clone(), x.clone()));
            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        });

        if args.subcomponent_shape_rep_seq.len() > 0 {
            // let f = ParryToSubcomponentsFilter2 { };
            // let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances);
            let res = OParryToSubcomponentFilter::query(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances, false, &());
            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        }

        args.subcomponent_shape_rep_seq.iter().for_each(|x| {
            // let f = ParryIntersectGroupFilter2::new(x.clone());
            // let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances);
            let res = OParryIntersectGroupFilter::query(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances, false, &OParryIntersectGroupFilterArgs::new(x.clone(), x.clone()));
            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        });

        OParryFilterOutput {
            selector: curr,
            duration: start.elapsed(),
            aux_datas,
        }
    }
}
pub type OwnedParryIntersectGroupSequenceFilter<T> = OwnedPairGroupQry<T, OParryIntersectGroupSequenceFilter>;

#[derive(Clone, Serialize, Deserialize)]
pub struct OParryIntersectGroupSequenceFilterArgs {
    shape_rep_seq: Vec<ParryShapeRep>,
    subcomponent_shape_rep_seq: Vec<ParryShapeRep>,
}
impl OParryIntersectGroupSequenceFilterArgs {
    pub fn new(shape_rep_seq: Vec<ParryShapeRep>, subcomponent_shape_rep_seq: Vec<ParryShapeRep>) -> Self {
        Self { shape_rep_seq, subcomponent_shape_rep_seq }
    }
}

pub struct OParryIntersectGroupSequenceFilterArgsCategory;
impl OPairGroupQryArgsCategoryTrait for OParryIntersectGroupSequenceFilterArgsCategory {
    type Args<T: AD> = OParryIntersectGroupSequenceFilterArgs;
    type QueryType = OParryIntersectGroupSequenceFilter;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

// DISTANCE SEQUENCE FILTER //

pub struct OParryDistanceGroupSequenceFilter;
impl OPairGroupQryTrait for OParryDistanceGroupSequenceFilter {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = OParryDistanceGroupSequenceFilterArgsCategory;
    type OutputCategory = OParryFilterOutputCategory;

    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, _freeze: bool, args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        let start = Instant::now();

        let mut curr = pair_selector.clone();
        let mut aux_datas = vec![];

        args.shape_rep_seq.iter().for_each(|x| {
            // let f = ParryDistanceGroupFilter2::new(x.clone(), args.parry_dis_mode.clone(), args.use_average_distance, args.distance_threshold);
            // let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances);
            let res = OParryDistanceGroupFilter::query(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances, false, &OParryDistanceGroupFilterArgs::new(x.clone(), x.clone(), args.parry_dis_mode.clone(), args.use_average_distance, args.distance_threshold));

            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        });

        if args.subcomponent_shape_rep_seq.len() > 0 {
            // let f = ParryToSubcomponentsFilter2 { };
            // let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances);
            let res = OParryToSubcomponentFilter::query(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances, false, &());
            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        }

        args.subcomponent_shape_rep_seq.iter().for_each(|x| {
            // let f = ParryDistanceGroupFilter2::new(x.clone(), args.parry_dis_mode.clone(), args.use_average_distance, args.distance_threshold);
            // let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances);
            let res = OParryDistanceGroupFilter::query(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips, pair_average_distances, false, &OParryDistanceGroupFilterArgs::new(x.clone(), x.clone(), args.parry_dis_mode.clone(), args.use_average_distance, args.distance_threshold));
            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        });

        OParryFilterOutput {
            selector: curr,
            duration: start.elapsed(),
            aux_datas,
        }
    }
}
pub type OwnedParryDistanceGroupSequenceFilter<T> = OwnedPairGroupQry<T, OParryDistanceGroupSequenceFilter>;

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct OParryDistanceGroupSequenceFilterArgs<T: AD> {
    shape_rep_seq: Vec<ParryShapeRep>,
    subcomponent_shape_rep_seq: Vec<ParryShapeRep>,
    #[serde_as(as = "SerdeAD<T>")]
    distance_threshold: T,
    use_average_distance: bool,
    parry_dis_mode: ParryDisMode
}
impl<T: AD> OParryDistanceGroupSequenceFilterArgs<T> {
    pub fn new(shape_rep_seq: Vec<ParryShapeRep>, subcomponent_shape_rep_seq: Vec<ParryShapeRep>, distance_threshold: T, use_average_distance: bool, parry_dis_mode: ParryDisMode) -> Self {
        Self { shape_rep_seq, subcomponent_shape_rep_seq, distance_threshold, use_average_distance, parry_dis_mode }
    }
}

pub struct OParryDistanceGroupSequenceFilterArgsCategory;
impl OPairGroupQryArgsCategoryTrait for OParryDistanceGroupSequenceFilterArgsCategory {
    type Args<T: AD> = OParryDistanceGroupSequenceFilterArgs<T>;
    type QueryType = OParryDistanceGroupSequenceFilter;
}

/*
pub struct PairGroupQryArgsCategoryParryDistanceSequenceFilterConverter;
impl ADConvertableTrait for PairGroupQryArgsCategoryParryDistanceSequenceFilterConverter {
    type ConvertableType<T: AD> = OParryDistanceGroupSequenceFilterArgs<T>;

    fn convert_to_other_ad_type<T1: AD, T2: AD>(input: &Self::ConvertableType<T1>) -> Self::ConvertableType<T2> {
        let json_str = input.to_json_string();
        Self::ConvertableType::<T2>::from_json_string(&json_str)
    }
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

// EMPTY FILTER //

pub struct EmptyParryFilter;
impl OPairGroupQryTrait for EmptyParryFilter {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = ();
    type OutputCategory = OParryFilterOutputCategory;

    #[inline(always)]
    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(_shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _poses_a: &Vec<P>, _poses_b: &Vec<P>, pair_selector: &Self::SelectorType, _pair_skips: &S, _pair_average_distances: &A, _freeze: bool, _args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        let start = Instant::now();
        OParryFilterOutput {
            selector: pair_selector.clone(),
            duration: start.elapsed(),
            aux_datas: vec![],
        }
    }
}
pub type OwnedEmptyParryFilter<T> = OwnedPairGroupQry<T, EmptyParryFilter>;

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
// ROBOT STATE DEPENDENT FILTER

pub struct ParryRobotStateDependentFilter<FQ>(PhantomData<FQ>) where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>;
impl<FQ> OPairGroupQryTrait for ParryRobotStateDependentFilter<FQ>
    where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>
{
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = ParryPairSelector;
    type ArgsCategory = PairGroupQryArgsCategoryParryRobotStateDependentFilter<FQ>;
    type OutputCategory = ParryFilterOutput;

    fn query<'a, T: AD, P: O3DPose<T>, S: PairSkipsTrait, A: PairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, freeze: bool, args: &<Self::ArgsCategory as PairGroupQryArgsCategory>::Args<'a, T>) -> <Self::OutputCategory as PairGroupQryOutputCategory>::Output<T, P> {
        todo!()
    }
}

pub struct ParryRobotStateDependentFilterArgs<'a, T: AD, FQ>
    where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>
{
    curr_state: RwLock<Vec<f64>>,
    last_checked_state: RwLock<Vec<f64>>,
    linf_cutoff_for_check: f64,
    curr_selector: RwLock<ParryPairSelector>,
    filter_query: OwnedPairGroupQry<'a, T, FQ>
}

pub struct PairGroupQryArgsCategoryParryRobotStateDependentFilter<FQ>(PhantomData<FQ>)
    where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>;
impl<FQ> PairGroupQryArgsCategory for PairGroupQryArgsCategoryParryRobotStateDependentFilter<FQ>
    where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>
{
    type Args<T: AD> = ParryRobotStateDependentFilterArgs<'a, T, FQ>;
    type QueryType = ParryRobotStateDependentFilter<FQ>;
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

// PROXIMITY LOSS FUNCTIONS //

/*
pub trait ProximityLossFunctionTrait : Serialize + DeserializeOwned {
    fn loss<T: AD>(&self, distance: T, cutoff: T) -> T;
}

#[derive(Serialize, Deserialize)]
pub struct ProximityLossFunctionHinge;
impl ProximityLossFunctionHinge {
    pub fn new() -> Self {
        Self {}
    }
}
impl ProximityLossFunctionTrait for ProximityLossFunctionHinge {
    #[inline(always)]
    fn loss<T: AD>(&self, distance: T, cutoff: T) -> T {
        return if distance > cutoff {
            T::zero()
        } else {
            -cutoff.recip() * distance + T::one()
        }
    }
}
*/

pub struct OParryProximityOutput<T: AD> {
    proximity_objective_value: T,
    duration: Duration,
    aux_datas: Vec<ParryOutputAuxData>
}
impl<T: AD> OParryProximityOutput<T> {
    #[inline(always)]
    pub fn proximity_value(&self) -> T {
        self.proximity_objective_value
    }
    #[inline(always)]
    pub fn duration(&self) -> Duration {
        self.duration
    }
    #[inline(always)]
    pub fn aux_datas(&self) -> &Vec<ParryOutputAuxData> {
        &self.aux_datas
    }
}

pub trait ToParryProximityOutputTrait<T: AD> {
    fn get_proximity_objective_value(&self, cutoff: T, p_norm: T, loss_function: OProximityLossFunction) -> T;
}
impl<T: AD> ToParryProximityOutputTrait<T> for () {
    fn get_proximity_objective_value(&self, _cutoff: T, _p_norm: T, _loss_function: OProximityLossFunction) -> T {
        T::zero()
    }
}

#[derive(Clone, Debug, Copy, Serialize, Deserialize)]
pub enum OProximityLossFunction {
    Hinge
}
impl OProximityLossFunction {
    #[inline(always)]
    pub fn loss<T: AD>(&self, distance: T, cutoff: T) -> T {
        return match self {
            OProximityLossFunction::Hinge => {
                if distance > cutoff {
                    T::zero()
                } else {
                    -cutoff.recip() * distance + T::one()
                }
            }
        }
    }
}

pub struct ToParryProximityOutputCategory;
impl OPairGroupQryOutputCategoryTrait for ToParryProximityOutputCategory {
    type Output<T: AD, P: O3DPose<T>> = Box<dyn ToParryProximityOutputTrait<T>>;
}

pub struct EmptyToParryProximity;
impl OPairGroupQryTrait for EmptyToParryProximity {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = OParryPairSelector;
    type ArgsCategory = ();
    type OutputCategory = ToParryProximityOutputCategory;

    #[inline(always)]
    fn query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>>(_shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, _poses_a: &Vec<P>, _poses_b: &Vec<P>, _pair_selector: &Self::SelectorType, _pair_skips: &S, _pair_average_distances: &A, _freeze: bool, _args: &<Self::ArgsCategory as OPairGroupQryArgsCategoryTrait>::Args<T>) -> <Self::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P> {
        Box::new(())
    }
}

pub type OwnedEmptyToProximityQry<T> = OwnedPairGroupQry<T, EmptyToParryProximity>;

////////////////////////////////////////////////////////////////////////////////////////////////////

#[inline(always)]
pub fn get_parry_ids_from_shape_pair<T: AD, P: O3DPose<T>>(shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep1: &ParryShapeRep, parry_shape_rep2: &ParryShapeRep) -> (u64, u64) {
    return match parry_qry_shape_type {
        ParryQryShapeType::Standard => { (shape_a.base_shape.id_from_shape_rep(parry_shape_rep1), shape_b.base_shape.id_from_shape_rep(parry_shape_rep2)) }
        ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx, shape_b_subcomponent_idx } => {
            (shape_a.convex_subcomponents[*shape_a_subcomponent_idx].id_from_shape_rep(parry_shape_rep1), shape_b.convex_subcomponents[*shape_b_subcomponent_idx].id_from_shape_rep(parry_shape_rep2))
        }
    }
}

#[inline(always)]
pub fn get_all_parry_pairs_idxs<T: AD, P: O3DPose<T>>(shape_group_a: &Vec<OParryShape<T, P>>, shape_group_b: &Vec<OParryShape<T, P>>, half_pairs: bool, subcomponents: bool) -> Vec<OParryPairIdxs> {
    let mut out = vec![];

    for i in 0..shape_group_a.len() {
        'l: for j in 0..shape_group_b.len() {
            if half_pairs { if i >= j { continue 'l; } }
            if subcomponents {
                let shape_a = &shape_group_a[i];
                let shape_b = &shape_group_b[j];
                let num_subcomponents_a = shape_a.convex_subcomponents.len();
                let num_subcomponents_b = shape_b.convex_subcomponents.len();
                for k in 0..num_subcomponents_a {
                    for l in 0..num_subcomponents_b {
                        out.push(OParryPairIdxs::ShapeSubcomponents((i, k), (j, l)));
                    }
                }
            } else {
                out.push(OParryPairIdxs::Shapes(i, j));
            }
        }
    }

    out
}

#[inline]
pub(crate) fn parry_generic_pair_group_query<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, O: AsAny, F, Termination>(shape_group_a: &Vec<OParryShape<T, P>>,
                                                                                                                 shape_group_b: &Vec<OParryShape<T, P>>,
                                                                                                                 poses_a: &Vec<P>,
                                                                                                                 poses_b: &Vec<P>,
                                                                                                                 pair_selector: &OParryPairSelector,
                                                                                                                 parry_shape_rep1: &ParryShapeRep,
                                                                                                                 parry_shape_rep2: &ParryShapeRep,
                                                                                                                 pair_skips: &S,
                                                                                                                 for_filter: bool,
                                                                                                                 f: F,
                                                                                                                 termination: Termination) -> (Vec<OParryPairGroupOutputWrapper<O>>, usize)
    where F: Fn(&OParryShape<T, P>, &OParryShape<T, P>, &P, &P, &ParryQryShapeType, &ParryShapeRep, &ParryShapeRep) -> O,
          Termination: Fn(&O) -> bool
{
    let mut out_vec = vec![];
    let mut count = 0;

    match pair_selector {
        OParryPairSelector::AllPairs => {
            'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                'l2: for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                    let id_a = shape_a.base_shape.id_from_shape_rep(parry_shape_rep1);
                    let id_b = shape_b.base_shape.id_from_shape_rep(parry_shape_rep2);
                    // let ids = parry_pair_idxs_to_shape_ids(shape_group_a, shape_group_b, &ParryPairIdxs::Shapes(i, j), parry_shape_rep);
                    if decide_skip_generic(id_a, id_b, pair_skips, for_filter) { continue 'l2; }
                    count += 1;
                    let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::Standard, parry_shape_rep1, parry_shape_rep2);
                    let terminate = termination(&o);
                    out_vec.push(OParryPairGroupOutputWrapper { data: o, pair_ids: (id_a, id_b), pair_idxs: OParryPairIdxs::Shapes(i, j) });
                    if terminate { break 'l; }
                }
            }
        }
        OParryPairSelector::HalfPairs => {
            'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                'l2: for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                    if i < j {
                        let id_a = shape_a.base_shape.id_from_shape_rep(parry_shape_rep1);
                        let id_b = shape_b.base_shape.id_from_shape_rep(parry_shape_rep2);
                        if decide_skip_generic(id_a, id_b, pair_skips, for_filter) { continue 'l2; }
                        count += 1;
                        let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::Standard, parry_shape_rep1, parry_shape_rep2);
                        let terminate = termination(&o);
                        out_vec.push(OParryPairGroupOutputWrapper { data: o, pair_ids: (id_a, id_b), pair_idxs: OParryPairIdxs::Shapes(i, j) });
                        if terminate { break 'l; }
                    }
                }
            }
        }
        OParryPairSelector::PairsByIdxs(idx_pairs) => {
            'l: for idx_pair in idx_pairs {
                match idx_pair {
                    OParryPairIdxs::Shapes(idx_pair0, idx_pair1) => {
                        let idx0 = *idx_pair0;
                        let idx1 = *idx_pair1;
                        let shape_a = &shape_group_a[idx0];
                        let shape_b = &shape_group_b[idx1];
                        let pose_a = &poses_a[idx0];
                        let pose_b = &poses_b[idx1];

                        let id_a = shape_a.base_shape.id_from_shape_rep(parry_shape_rep1);
                        let id_b = shape_b.base_shape.id_from_shape_rep(parry_shape_rep2);
                        if decide_skip_generic(id_a, id_b, pair_skips, for_filter) { continue 'l; }

                        count += 1;
                        let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::Standard, parry_shape_rep1, parry_shape_rep2);
                        let terminate = termination(&o);
                        out_vec.push(OParryPairGroupOutputWrapper { data: o, pair_ids: (id_a, id_b), pair_idxs: OParryPairIdxs::Shapes(idx0, idx1) });
                        if terminate { break 'l; }
                    }
                    OParryPairIdxs::ShapeSubcomponents(idx_pairs0, idx_pairs1) => {
                        let idxs0 = *idx_pairs0;
                        let idxs1 = *idx_pairs1;
                        let shape_a_idx = idxs0.0;
                        let shape_a_subcomponent_idx = idxs0.1;
                        let shape_b_idx = idxs1.0;
                        let shape_b_subcomponent_idx = idxs1.1;

                        let shape_a = &shape_group_a[shape_a_idx];
                        let shape_b = &shape_group_b[shape_b_idx];
                        let pose_a = &poses_a[shape_a_idx];
                        let pose_b = &poses_b[shape_b_idx];

                        let id_a = shape_a.convex_subcomponents[shape_a_subcomponent_idx].id_from_shape_rep(parry_shape_rep1);
                        let id_b = shape_b.convex_subcomponents[shape_b_subcomponent_idx].id_from_shape_rep(parry_shape_rep2);
                        if decide_skip_generic(id_a, id_b, pair_skips, for_filter) { continue 'l; }

                        count += 1;
                        let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx, shape_b_subcomponent_idx }, parry_shape_rep1, parry_shape_rep2);
                        let terminate = termination(&o);
                        out_vec.push(OParryPairGroupOutputWrapper { data: o, pair_ids: (id_a, id_b), pair_idxs: OParryPairIdxs::ShapeSubcomponents((shape_a_idx, shape_a_subcomponent_idx), (shape_b_idx, shape_b_subcomponent_idx)) });
                        if terminate { break 'l; }
                    }
                }
            }
        }
        OParryPairSelector::AllPairsSubcomponents => {
            'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                    let num_subcomponents_a = shape_a.convex_subcomponents.len();
                    let num_subcomponents_b = shape_b.convex_subcomponents.len();

                    for k in 0..num_subcomponents_a {
                        let id_a = shape_a.convex_subcomponents.get(k).unwrap().id_from_shape_rep(parry_shape_rep1);
                        'l2: for l in 0..num_subcomponents_b {
                            let id_b = shape_b.convex_subcomponents.get(l).unwrap().id_from_shape_rep(parry_shape_rep2);
                            if decide_skip_generic(id_a, id_b, pair_skips, for_filter) { continue 'l2; }
                            count += 1;
                            let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx: k, shape_b_subcomponent_idx: l }, parry_shape_rep1, parry_shape_rep2);
                            let terminate = termination(&o);
                            out_vec.push(OParryPairGroupOutputWrapper { data: o, pair_ids: (id_a, id_b), pair_idxs: OParryPairIdxs::ShapeSubcomponents((i, k), (j, l)) });
                            if terminate { break 'l; }
                        }
                    }
                }
            }
        }
        OParryPairSelector::HalfPairsSubcomponents => {
            'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                    if i < j {
                        let num_subcomponents_a = shape_a.convex_subcomponents.len();
                        let num_subcomponents_b = shape_b.convex_subcomponents.len();

                        for k in 0..num_subcomponents_a {
                            let id_a = shape_a.convex_subcomponents.get(k).unwrap().id_from_shape_rep(parry_shape_rep1);
                            'l2: for l in 0..num_subcomponents_b {
                                let id_b = shape_b.convex_subcomponents.get(l).unwrap().id_from_shape_rep(parry_shape_rep2);
                                if decide_skip_generic(id_a, id_b, pair_skips, for_filter) { continue 'l2; }
                                count += 1;
                                let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx: k, shape_b_subcomponent_idx: l }, parry_shape_rep1, parry_shape_rep2);
                                let terminate = termination(&o);
                                out_vec.push(OParryPairGroupOutputWrapper { data: o, pair_ids: (id_a, id_b), pair_idxs: OParryPairIdxs::ShapeSubcomponents((i, k), (j, l)) });
                                if terminate { break 'l; }
                            }
                        }
                    }
                }
            }
        }
    }

    (out_vec, count)
}

#[inline(always)]
pub (crate) fn decide_skip_generic<S: OPairSkipsTrait>(id_a: u64, id_b: u64, pair_skips: &S, for_filter: bool) -> bool {
    if for_filter {
        let skip_reasons = &pair_skips.skip_reasons(id_a, id_b);
        match skip_reasons {
            None => { return false; }
            Some(skip_reasons) => {
                let r = skip_reasons.as_ref();
                if r.contains(&OSkipReason::NeverInCollision) { return true; }
            }
        }
    } else {
        if pair_skips.skip(id_a, id_b) { return true; }
    }
    return false;
}

pub (crate) fn parry_generic_pair_group_filter<T: AD, P: O3DPose<T>, S: OPairSkipsTrait, A: OPairAverageDistanceTrait<T>, Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=OParryPairSelector>, F>(qry: OwnedPairGroupQry<T, Q>,
                                                                                                                                                                                                                             shape_group_a: &Vec<OParryShape<T, P>>,
                                                                                                                                                                                                                             shape_group_b: &Vec<OParryShape<T, P>>,
                                                                                                                                                                                                                             poses_a: &Vec<P>,
                                                                                                                                                                                                                             poses_b: &Vec<P>,
                                                                                                                                                                                                                             pair_selector: &OParryPairSelector,
                                                                                                                                                                                                                             pair_skips: &S,
                                                                                                                                                                                                                             pair_average_distances: &A,
                                                                                                                                                                                                                             f: F) -> OParryFilterOutput
    where F: Fn(&<Q::OutputCategory as OPairGroupQryOutputCategoryTrait>::Output<T, P>) -> Vec<OParryPairIdxs>
{
    let start = Instant::now();

    let output = qry.query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, pair_skips, pair_average_distances, false);
    let parry_pair_idxs = f(&output);

    let selector = convert_parry_pair_idxs_to_parry_pair_selector(parry_pair_idxs);
    OParryFilterOutput {
        selector,
        duration: start.elapsed(),
        aux_datas: vec![ParryOutputAuxData { num_queries: 0, duration: start.elapsed() }],
    }
}

#[inline]
pub (crate) fn convert_parry_pair_idxs_to_parry_pair_selector(parry_pair_idxs: Vec<OParryPairIdxs>) -> OParryPairSelector {
    let mut a = vec![];

    parry_pair_idxs.iter().for_each(|x| {
        a.push(x.clone());
    });

    OParryPairSelector::PairsByIdxs(a)
}

#[inline(always)]
pub (crate) fn get_average_distance_option_from_shape_pair<T: AD, P: O3DPose<T>, A: OPairAverageDistanceTrait<T>>(use_average_distance: bool, shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep1: &ParryShapeRep, parry_shape_rep2: &ParryShapeRep, for_filter: bool, pair_average_distances: &A) -> Option<T> {
    return if use_average_distance {
        let ids = if for_filter {
            get_parry_ids_from_shape_pair(shape_a, shape_b, parry_qry_shape_type, &ParryShapeRep::Full, &ParryShapeRep::Full)
        } else {
            get_parry_ids_from_shape_pair(shape_a, shape_b, parry_qry_shape_type, parry_shape_rep1, parry_shape_rep2)
        };
        let a = pair_average_distances.average_distance(ids.0, ids.1);
        Some(a)
    } else {
        None
    };
}

/*
pub struct OParryPairGroupDefaultQry<T: AD, P: O3DPose<T>, Q: OPairQryTrait<T, P, ShapeTypeA=OParryShape<T, P>, ShapeTypeB=OParryShape<T, P>, Output : PartialOrd>, Termination: OPairGroupTermination<PairQryOutput = Q::Output>> {
    pair_query: Q,
    sort_outputs: bool,
    termination: Termination,
    phantom_data: PhantomData<(T, P)>
}
impl<T: AD, P: O3DPose<T>, Q: OPairQryTrait<T, P, ShapeTypeA=OParryShape<T, P>, ShapeTypeB=OParryShape<T, P>, Output: PartialOrd>, Termination: OPairGroupTermination<PairQryOutput = Q::Output>> OParryPairGroupDefaultQry<T, P, Q, Termination> {
    pub fn new(pair_query: Q, sort_outputs: bool, termination: Termination) -> Self {
        Self { pair_query, sort_outputs, termination, phantom_data: PhantomData::default() }
    }
}
impl<T: AD, P: O3DPose<T>, Q: OPairQryTrait<T, P, ShapeTypeA=OParryShape<T, P>, ShapeTypeB=OParryShape<T, P>, Output : PartialOrd> + 'static, Termination: OPairGroupTermination<PairQryOutput = Q::Output>> OPairGroupQryTrait<T, P> for OParryPairGroupDefaultQry<T, P, Q, Termination> {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Output = OParryPairGroupDefaultOutput<T, P, Q>;

    fn query(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &ParryPairSelector) -> Self::Output {
        let start = Instant::now();
        let mut count = 0;
        let mut outputs = vec![];

        match pair_selector {
            ParryPairSelector::AllPairs => {
                'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                    for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                        count += 1;
                        let output = self.pair_query.query(shape_a, shape_b, pose_a, pose_b);
                        let terminate = self.termination.terminate(&output);
                        outputs.push(PairGroupOutputWrapper { output, pair_idxs: (i, j) });
                        if terminate { break 'l; }
                    }
                }
            }
            ParryPairSelector::HalfPairs => {
                'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                    for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                        if i < j {
                            count += 1;
                            let output = self.pair_query.query(shape_a, shape_b, pose_a, pose_b);
                            let terminate = self.termination.terminate(&output);
                            outputs.push(PairGroupOutputWrapper { output, pair_idxs: (i, j) });
                            if terminate { break 'l; }
                        }
                    }
                }
            }
            ParryPairSelector::PairsByIdxs(idx_pairs) => {
                'l: for idx_pair in idx_pairs {
                    let idx_a = idx_pair.0;
                    let idx_b = idx_pair.1;

                    count += 1;
                    let output = self.pair_query.query(&shape_group_a[idx_a], &shape_group_b[idx_b], &poses_a[idx_a], &poses_b[idx_b]);
                    let terminate = self.termination.terminate(&output);
                    outputs.push(PairGroupOutputWrapper { output, pair_idxs: (idx_a, idx_b) });
                    if terminate { break 'l; }
                }
            }
            ParryPairSelector::PairSubcomponentsByIdxs(subcomponent_idx_pairs) => {
                'l: for subcomponent_idx_pair in subcomponent_idx_pairs {
                    let idxs_a = subcomponent_idx_pair.0;
                    let idxs_b = subcomponent_idx_pair.1;

                    let shape_idx_a = idxs_a.0;
                    let subcomponent_idx_a = idxs_a.1;

                    let shape_idx_b = idxs_b.0;
                    let subcomponent_idx_b = idxs_b.1;
                }
                todo!()
            }
        }

        if self.sort_outputs { outputs.sort_by(|x, y| x.output.partial_cmp(&y.output).unwrap()) }

        OParryPairGroupDefaultOutput {
            outputs,
            aux_data: ParryOutputAuxData { num_queries : count, duration: start.elapsed() },
        }
    }
}

pub struct OParryPairGroupDefaultOutput<T: AD, P: O3DPose<T>, Q: OPairQryTrait<T, P>> {
    outputs: Vec<PairGroupOutputWrapper<Q::Output>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD, P: O3DPose<T>, Q: OPairQryTrait<T, P>> OParryPairGroupDefaultOutput<T, P, Q> {
    #[inline(always)]
    pub fn outputs(&self) -> &Vec<PairGroupOutputWrapper<Q::Output>> {
        &self.outputs
    }
    #[inline(always)]
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

pub struct NeverTerminate<O>(PhantomData<O>);
impl<O> NeverTerminate<O> {
    pub fn new() -> Self {
        Self(PhantomData::default())
    }
}
impl<O: 'static> OPairGroupTermination for NeverTerminate<O> {
    type PairQryOutput = O;

    fn terminate(&self, _pair_query_output: &Self::PairQryOutput) -> bool {
        false
    }
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////
