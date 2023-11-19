use std::collections::HashMap;
use std::marker::PhantomData;
use std::time::{Duration, Instant};
use ad_trait::AD;
use ahash::AHashMap;
use as_any::AsAny;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_universal_hashmap::AHashMapWrapper;
use crate::pair_queries::{OPairQryTrait, ParryContactOutput, ParryContactQry, ParryContactWrtAverageOutput, ParryContactWrtAverageQry, ParryDisMode, ParryDistanceBoundsOutput, ParryDistanceBoundsQry, ParryDistanceLowerBoundOutput, ParryDistanceLowerBoundQry, ParryDistanceOutput, ParryDistanceQry, ParryDistanceUpperBoundOutput, ParryDistanceUpperBoundQry, ParryDistanceWrtAverageOutput, ParryDistanceWrtAverageQry, ParryIntersectOutput, ParryIntersectQry, ParryOutputAuxData, ParryQryShapeType, ParryShapeRep};
use crate::shape_queries::{ContactOutputTrait, DistanceOutputTrait, IntersectOutputTrait};
use crate::shapes::{OParryShape};

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait OPairGroupQryTrait<T: AD, P: O3DPose<T>> {
    type ShapeTypeA : AsAny;
    type ShapeTypeB : AsAny;
    type SelectorType : AsAny;
    type Args;
    type Output : AsAny;

    fn query<S: PairSkipsTrait>(shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, args: &Self::Args) -> Self::Output;
}

pub trait PairSkipsTrait {
    fn skip(&self, shape_a_id: u64, shape_b_id: u64) -> bool;
}
impl PairSkipsTrait for () {
    fn skip(&self, _shape_a_id: u64, _shape_b_id: u64) -> bool {
        false
    }
}
impl PairSkipsTrait for HashMap<(u64, u64), ()> {
    fn skip(&self, shape_a_id: u64, shape_b_id: u64) -> bool {
        let res = self.get(&(shape_a_id, shape_b_id));
        return match res {
            None => { false }
            Some(_) => { true }
        }
    }
}
impl PairSkipsTrait for AHashMap<(u64, u64), ()> {
    #[inline(always)]
    fn skip(&self, shape_a_id: u64, shape_b_id: u64) -> bool {
        let res = self.get(&(shape_a_id, shape_b_id));
        return match res {
            None => { false }
            Some(_) => { true }
        }
    }
}
impl PairSkipsTrait for AHashMapWrapper<(u64, u64), ()> {
    #[inline(always)]
    fn skip(&self, shape_a_id: u64, shape_b_id: u64) -> bool {
        let res = self.hashmap.get(&(shape_a_id, shape_b_id));
        return match res {
            None => { false }
            Some(_) => { true }
        }
    }
}

pub trait PairAverageDistanceTrait<T: AD> {
    fn average_distance(&self, shape_a_id: u64, shape_b_id: u64) -> T;
}
impl<T: AD> PairAverageDistanceTrait<T> for AHashMapWrapper<(u64, u64), T> {
    #[inline(always)]
    fn average_distance(&self, shape_a_id: u64, shape_b_id: u64) -> T {
        *self.hashmap.get(&(shape_a_id, shape_b_id)).unwrap()
    }
}
impl<T: AD> PairAverageDistanceTrait<T> for () {
    #[inline(always)]
    fn average_distance(&self, _shape_a_id: u64, _shape_b_id: u64) -> T {
        T::one()
    }
}

#[derive(Clone, Debug)]
pub enum ParryPairSelector {
    AllPairs,
    HalfPairs,
    AllPairsSubcomponents,
    HalfPairsSubcomponents,
    PairsByIdxs(Vec<ParryPairIdxs>)
    // PairSubcomponentsByIdxs(Vec<((usize, usize), (usize, usize))>)
}
pub struct ParryPairGroupOutputWrapper<O> {
    data: O,
    pair_ids: (u64, u64),
    pair_idxs: ParryPairIdxs
}
impl<O> ParryPairGroupOutputWrapper<O> {
    #[inline(always)]
    pub fn data(&self) -> &O {
        &self.data
    }
    #[inline(always)]
    pub fn pair_idxs(&self) -> &ParryPairIdxs {
        &self.pair_idxs
    }
    #[inline(always)]
    pub fn pair_ids(&self) -> (u64, u64) {
        self.pair_ids
    }
}
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum ParryPairIdxs {
    Shapes(usize, usize),
    ShapeSubcomponents((usize, usize), (usize, usize))
}
impl ParryPairIdxs {
    pub fn to_parry_shape_idxs(&self) -> (ParryShapeIdx, ParryShapeIdx) {
        match self {
            ParryPairIdxs::Shapes(x, y) => { (ParryShapeIdx::Shape(*x), ParryShapeIdx::Shape(*y)) }
            ParryPairIdxs::ShapeSubcomponents(x, y) => { (ParryShapeIdx::ShapeSubcomponent(*x), ParryShapeIdx::ShapeSubcomponent(*y)) }
        }
    }
}

#[derive(Debug, Clone)]
pub enum ParryShapeIdx {
    Shape(usize),
    ShapeSubcomponent((usize, usize))
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct ParryIntersectGroupQry;
impl<T: AD, P: O3DPose<T>> OPairGroupQryTrait<T, P> for ParryIntersectGroupQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Args = ParryIntersectGroupArgs;
    type Output = ParryIntersectGroupOutput;

    fn query<S: PairSkipsTrait>(shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, args: &Self::Args) -> Self::Output {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryIntersectOutput {
            ParryIntersectQry::query(shape_a, shape_b, pose_a, pose_b, &(parry_qry_shape_type.clone(), parry_shape_rep.clone()))
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

        let (mut outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, f, termination);

        outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        ParryIntersectGroupOutput {
            intersect: outputs[0].data.intersect(),
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}

pub struct ParryIntersectGroupArgs {
    parry_shape_rep: ParryShapeRep,
    terminate_on_first_intersection: bool
}
impl ParryIntersectGroupArgs {
    pub fn new(parry_shape_rep: ParryShapeRep, terminate_on_first_intersection: bool) -> Self {
        Self { parry_shape_rep, terminate_on_first_intersection }
    }
}

pub struct ParryIntersectGroupOutput {
    intersect: bool,
    outputs: Vec<ParryPairGroupOutputWrapper<ParryIntersectOutput>>,
    aux_data: ParryOutputAuxData
}
impl ParryIntersectGroupOutput {
    pub fn intersect(&self) -> bool {
        self.intersect
    }
    pub fn outputs(&self) -> &Vec<ParryPairGroupOutputWrapper<ParryIntersectOutput>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct ParryDistanceGroupQry;
impl<T: AD, P: O3DPose<T>> OPairGroupQryTrait<T, P> for ParryDistanceGroupQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Args = ParryDistanceGroupArgs<T>;
    type Output = ParryDistanceGroupOutput<T>;

    fn query<S: PairSkipsTrait>(shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, args: &Self::Args) -> Self::Output {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryDistanceOutput<T> {
            ParryDistanceQry::query(shape_a, shape_b, pose_a, pose_b, &(args.parry_dis_mode.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone()))
        };

        let termination = |o: &ParryDistanceOutput<T>| {
            return o.distance() <= args.termination_distance_threshold
        };

        let (mut outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, f, termination);

        outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        ParryDistanceGroupOutput {
            min_dis: outputs[0].data.distance(),
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}

pub struct ParryDistanceGroupArgs<T: AD> {
    parry_shape_rep: ParryShapeRep,
    parry_dis_mode: ParryDisMode,
    termination_distance_threshold: T
}
impl<T: AD> ParryDistanceGroupArgs<T> {
    pub fn new(parry_shape_rep: ParryShapeRep, parry_dis_mode: ParryDisMode, termination_distance_threshold: T) -> Self {
        Self { parry_shape_rep, parry_dis_mode, termination_distance_threshold }
    }
}

pub struct ParryDistanceGroupOutput<T: AD> {
    min_dis: T,
    outputs: Vec<ParryPairGroupOutputWrapper<ParryDistanceOutput<T>>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceGroupOutput<T> {
    pub fn min_dis(&self) -> &T {
        &self.min_dis
    }
    pub fn outputs(&self) -> &Vec<ParryPairGroupOutputWrapper<ParryDistanceOutput<T>>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct ParryDistanceWrtAverageGroupQry<'a>(PhantomData<&'a ()>);
impl<'a, T: AD, P: O3DPose<T>> OPairGroupQryTrait<T, P> for ParryDistanceWrtAverageGroupQry<'a> {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Args = ParryDistanceWrtAverageGroupArgs<'a, T>;
    type Output = ParryDistanceWrtAverageGroupOutput<T>;

    fn query<S: PairSkipsTrait>(shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, args: &Self::Args) -> Self::Output {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryDistanceWrtAverageOutput<T> {
            let ids = get_parry_ids_from_shape_pair(shape_a, shape_b, parry_qry_shape_type, parry_shape_rep);
            let average_dis = args.average_distances.hashmap.get(&(ids)).expect("error");
            ParryDistanceWrtAverageQry::query(shape_a, shape_b, pose_a, pose_b, &(args.parry_dis_mode.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone(), average_dis.clone()))
        };

        let termination = |o: &ParryDistanceWrtAverageOutput<T>| {
            return o.distance_wrt_average <= args.termination_distance_threshold
        };

        let (mut outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, f, termination);

        outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        ParryDistanceWrtAverageGroupOutput {
            min_dis_wrt_average: outputs[0].data.distance_wrt_average,
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}

pub struct ParryDistanceWrtAverageGroupArgs<'a, T: AD> {
    parry_shape_rep: ParryShapeRep,
    parry_dis_mode: ParryDisMode,
    average_distances: &'a AHashMapWrapper<(u64, u64), T>,
    termination_distance_threshold: T
}
impl<'a, T: AD> ParryDistanceWrtAverageGroupArgs<'a, T> {
    pub fn new(parry_shape_rep: ParryShapeRep, parry_dis_mode: ParryDisMode, average_distances: &'a AHashMapWrapper<(u64, u64), T>, termination_distance_threshold: T) -> Self {
        Self { parry_shape_rep, parry_dis_mode, average_distances, termination_distance_threshold }
    }
}

pub struct ParryDistanceWrtAverageGroupOutput<T: AD> {
    min_dis_wrt_average: T,
    outputs: Vec<ParryPairGroupOutputWrapper<ParryDistanceWrtAverageOutput<T>>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceWrtAverageGroupOutput<T> {
    pub fn min_dis_wrt_average(&self) -> &T {
        &self.min_dis_wrt_average
    }
    pub fn outputs(&self) -> &Vec<ParryPairGroupOutputWrapper<ParryDistanceWrtAverageOutput<T>>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct ParryContactGroupQry;
impl<T: AD, P: O3DPose<T>> OPairGroupQryTrait<T, P> for ParryContactGroupQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Args = ParryContactGroupArgs<T>;
    type Output = ParryContactGroupOutput<T>;

    fn query<S: PairSkipsTrait>(shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, args: &Self::Args) -> Self::Output {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryContactOutput<T> {
            ParryContactQry::query(shape_a, shape_b, pose_a, pose_b, &(args.contact_threshold.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone()))
        };

        let termination = |o: &ParryContactOutput<T>| {
            let signed_distance = o.signed_distance();
            return match &signed_distance {
                None => { false }
                Some(signed_distance) => { *signed_distance <= args.termination_distance_threshold }
            }
        };

        let (mut outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, f, termination);

        outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        ParryContactGroupOutput {
            min_signed_dis: outputs[0].data.signed_distance(),
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}

pub struct ParryContactGroupArgs<T: AD> {
    parry_shape_rep: ParryShapeRep,
    contact_threshold: T,
    termination_distance_threshold: T
}
impl<T: AD> ParryContactGroupArgs<T> {
    pub fn new(parry_shape_rep: ParryShapeRep, contact_threshold: T, termination_distance_threshold: T) -> Self {
        Self { parry_shape_rep, contact_threshold, termination_distance_threshold }
    }
}

pub struct ParryContactGroupOutput<T: AD> {
    min_signed_dis: Option<T>,
    outputs: Vec<ParryPairGroupOutputWrapper<ParryContactOutput<T>>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD> ParryContactGroupOutput<T> {
    pub fn min_signed_dis(&self) -> &Option<T> {
        &self.min_signed_dis
    }
    pub fn outputs(&self) -> &Vec<ParryPairGroupOutputWrapper<ParryContactOutput<T>>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct ParryContactWrtAverageGroupQry<'a>(PhantomData<&'a ()>);
impl<'a, T: AD, P: O3DPose<T>> OPairGroupQryTrait<T, P> for ParryContactWrtAverageGroupQry<'a> {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Args = ParryContactWrtAverageGroupArgs<'a, T>;
    type Output = ParryContactWrtAverageGroupOutput<T>;

    fn query<S: PairSkipsTrait>(shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, args: &Self::Args) -> Self::Output {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryContactWrtAverageOutput<T> {
            let ids = get_parry_ids_from_shape_pair(shape_a, shape_b, parry_qry_shape_type, parry_shape_rep);
            let average_dis = args.average_distances.hashmap.get(&(ids)).expect("error");
            ParryContactWrtAverageQry::query(shape_a, shape_b, pose_a, pose_b, &(args.contact_threshold, parry_qry_shape_type.clone(), parry_shape_rep.clone(), average_dis.clone()))
        };

        let termination = |o: &ParryContactWrtAverageOutput<T>| {
            return match o.distance_wrt_average {
                None => { false }
                Some(d) => { d <= args.termination_distance_threshold }
            };
        };

        let (mut outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, f, termination);

        outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        ParryContactWrtAverageGroupOutput {
            min_signed_dis_wrt_average: outputs[0].data.distance_wrt_average,
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}

pub struct ParryContactWrtAverageGroupArgs<'a, T: AD> {
    parry_shape_rep: ParryShapeRep,
    contact_threshold: T,
    average_distances: &'a AHashMapWrapper<(u64, u64), T>,
    termination_distance_threshold: T
}
impl<'a, T: AD> ParryContactWrtAverageGroupArgs<'a, T> {
    pub fn new(parry_shape_rep: ParryShapeRep, contact_threshold: T, average_distances: &'a AHashMapWrapper<(u64, u64), T>, termination_distance_threshold: T) -> Self {
        Self { parry_shape_rep, contact_threshold, average_distances, termination_distance_threshold }
    }
}

pub struct ParryContactWrtAverageGroupOutput<T: AD> {
    min_signed_dis_wrt_average: Option<T>,
    outputs: Vec<ParryPairGroupOutputWrapper<ParryContactWrtAverageOutput<T>>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD> ParryContactWrtAverageGroupOutput<T> {
    pub fn min_signed_dis_wrt_average(&self) -> &Option<T> {
        &self.min_signed_dis_wrt_average
    }
    pub fn outputs(&self) -> &Vec<ParryPairGroupOutputWrapper<ParryContactWrtAverageOutput<T>>> {
        &self.outputs
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct ParryDistanceLowerBoundGroupQry;
impl<T: AD, P: O3DPose<T>> OPairGroupQryTrait<T, P> for ParryDistanceLowerBoundGroupQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Args = ParryDistanceLowerBoundGroupArgs;
    type Output = ParryDistanceLowerBoundGroupOutput<T>;

    fn query<S: PairSkipsTrait>(shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, args: &Self::Args) -> Self::Output {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryDistanceLowerBoundOutput<T> {
            // ParryContactQry::query(shape_a, shape_b, pose_a, pose_b, &(args.contact_threshold.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone()))
            ParryDistanceLowerBoundQry::query(shape_a, shape_b, pose_a, pose_b, &(args.parry_dis_mode.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone()))
        };

        let termination = |_o: &ParryDistanceLowerBoundOutput<T>| {
            false
        };

        let (outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, f, termination);

        // outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        ParryDistanceLowerBoundGroupOutput {
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}

pub struct ParryDistanceLowerBoundGroupArgs {
    parry_dis_mode: ParryDisMode,
    parry_shape_rep: ParryShapeRep
}
impl ParryDistanceLowerBoundGroupArgs {
    pub fn new(parry_dis_mode: ParryDisMode, parry_shape_rep: ParryShapeRep) -> Self {
        Self { parry_dis_mode, parry_shape_rep }
    }
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

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct ParryDistanceUpperBoundGroupQry;
impl<T: AD, P: O3DPose<T>> OPairGroupQryTrait<T, P> for ParryDistanceUpperBoundGroupQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Args = ParryDistanceUpperBoundGroupArgs;
    type Output = ParryDistanceUpperBoundGroupOutput<T>;

    fn query<S: PairSkipsTrait>(shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, args: &Self::Args) -> Self::Output {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryDistanceUpperBoundOutput<T> {
            // ParryContactQry::query(shape_a, shape_b, pose_a, pose_b, &(args.contact_threshold.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone()))
            ParryDistanceUpperBoundQry::query(shape_a, shape_b, pose_a, pose_b, &(args.parry_dis_mode.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone()))
        };

        let termination = |_o: &ParryDistanceUpperBoundOutput<T>| {
            false
        };

        let (outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, f, termination);

        // outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        ParryDistanceUpperBoundGroupOutput {
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}

pub struct ParryDistanceUpperBoundGroupArgs {
    parry_dis_mode: ParryDisMode,
    parry_shape_rep: ParryShapeRep
}
impl ParryDistanceUpperBoundGroupArgs {
    pub fn new(parry_dis_mode: ParryDisMode, parry_shape_rep: ParryShapeRep) -> Self {
        Self { parry_dis_mode, parry_shape_rep }
    }
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

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct ParryDistanceBoundsGroupQry;
impl<T: AD, P: O3DPose<T>> OPairGroupQryTrait<T, P> for ParryDistanceBoundsGroupQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Args = ParryDistanceBoundsGroupArgs;
    type Output = ParryDistanceBoundsGroupOutput<T>;

    fn query<S: PairSkipsTrait>(shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, args: &Self::Args) -> Self::Output {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryDistanceBoundsOutput<T> {
            // ParryContactQry::query(shape_a, shape_b, pose_a, pose_b, &(args.contact_threshold.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone()))
            ParryDistanceBoundsQry::query(shape_a, shape_b, pose_a, pose_b, &(args.parry_dis_mode.clone(), parry_qry_shape_type.clone(), parry_shape_rep.clone()))
        };

        let termination = |_o: &ParryDistanceBoundsOutput<T>| {
            false
        };

        let (outputs, num_queries) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, f, termination);

        // outputs.sort_by(|x, y| x.data.partial_cmp(&y.data).unwrap());

        ParryDistanceBoundsGroupOutput {
            outputs,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        }
    }
}

pub struct ParryDistanceBoundsGroupArgs {
    parry_dis_mode: ParryDisMode,
    parry_shape_rep: ParryShapeRep
}
impl ParryDistanceBoundsGroupArgs {
    pub fn new(parry_dis_mode: ParryDisMode, parry_shape_rep: ParryShapeRep) -> Self {
        Self { parry_dis_mode, parry_shape_rep }
    }
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

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait OPairGroupFilterTrait<T: AD, P: O3DPose<T>> {
    type ShapeTypeA : AsAny;
    type ShapeTypeB : AsAny;
    type SelectorType : AsAny;
    type Output : AsAny;

    fn pair_group_filter<S: PairSkipsTrait>(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S) -> Self::Output;
}
pub trait OParryPairGroupFilterTrait<T: AD, P: O3DPose<T>> : OPairGroupFilterTrait<T, P, Output : AsParryFilterOutputTrait + AsAny> { }
pub trait AsParryFilterOutputTrait {
    fn as_parry_filter_output(&self) -> &ParryFilterOutput;
}

pub struct ParryDistanceGroupFilter<T: AD> {
    parry_shape_rep: ParryShapeRep,
    parry_dis_mode: ParryDisMode,
    distance_threshold: T
}
impl<T: AD> ParryDistanceGroupFilter<T> {
    pub fn new(parry_shape_rep: ParryShapeRep, parry_dis_mode: ParryDisMode, distance_threshold: T) -> Self {
        Self { parry_shape_rep, parry_dis_mode, distance_threshold }
    }
}
impl<T: AD, P: O3DPose<T>> OPairGroupFilterTrait<T, P> for ParryDistanceGroupFilter<T> {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Output = ParryFilterOutput;

    fn pair_group_filter<S: PairSkipsTrait>(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S) -> Self::Output {
        let start = Instant::now();

        let output = ParryDistanceGroupQry::query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, pair_skips, &ParryDistanceGroupArgs::new(self.parry_shape_rep.clone(), self.parry_dis_mode.clone(), T::constant(f64::MIN)));

        let mut a = vec![];

        output.outputs.iter().for_each(|x| {
            if x.data.distance < self.distance_threshold {
                a.push(x.pair_idxs.clone());
            }
        });

        let selector = convert_parry_pair_idxs_to_parry_pair_selector(a);

        ParryFilterOutput {
            selector,
            duration: start.elapsed(),
            aux_datas: vec![ParryOutputAuxData { num_queries: 0, duration: start.elapsed() }],
        }
    }
}
impl<T: AD, P: O3DPose<T>> OParryPairGroupFilterTrait<T, P> for ParryDistanceGroupFilter<T> { }

pub struct ParryDistanceWrtAverageGroupFilter<'a, T: AD> {
    parry_shape_rep: ParryShapeRep,
    parry_dis_mode: ParryDisMode,
    average_distances: &'a AHashMapWrapper<(u64, u64), T>,
    distance_wrt_average_threshold: T
}
impl<'a, T: AD> ParryDistanceWrtAverageGroupFilter<'a, T> {
    pub fn new(parry_shape_rep: ParryShapeRep, parry_dis_mode: ParryDisMode, average_distances: &'a AHashMapWrapper<(u64, u64), T>, distance_wrt_average_threshold: T) -> Self {
        Self { parry_shape_rep, parry_dis_mode, average_distances, distance_wrt_average_threshold }
    }
}
impl<'a, T: AD, P: O3DPose<T>> OPairGroupFilterTrait<T, P> for ParryDistanceWrtAverageGroupFilter<'a, T> {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Output = ParryFilterOutput;

    fn pair_group_filter<S: PairSkipsTrait>(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S) -> Self::Output {
        let start = Instant::now();

        let output = ParryDistanceWrtAverageGroupQry::query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, pair_skips, &ParryDistanceWrtAverageGroupArgs::new(self.parry_shape_rep.clone(), self.parry_dis_mode.clone(), self.average_distances, T::constant(f64::MIN)));

        let mut a = vec![];

        output.outputs.iter().for_each(|x| {
            if x.data.distance_wrt_average < self.distance_wrt_average_threshold {
                a.push(x.pair_idxs.clone());
            }
        });

        let selector = convert_parry_pair_idxs_to_parry_pair_selector(a);

        ParryFilterOutput {
            selector,
            duration: start.elapsed(),
            aux_datas: vec![ParryOutputAuxData { num_queries: 0, duration: start.elapsed() }],
        }
    }
}
impl<'a, T: AD, P: O3DPose<T>> OParryPairGroupFilterTrait<T, P> for ParryDistanceWrtAverageGroupFilter<'a, T> { }

pub struct ParryIntersectGroupFilter {
    parry_shape_rep: ParryShapeRep
}
impl ParryIntersectGroupFilter {
    pub fn new(parry_shape_rep: ParryShapeRep) -> Self {
        Self { parry_shape_rep }
    }
}
impl<T: AD, P: O3DPose<T>> OPairGroupFilterTrait<T, P> for ParryIntersectGroupFilter {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Output = ParryFilterOutput;

    fn pair_group_filter<S: PairSkipsTrait>(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S) -> Self::Output {
        let start = Instant::now();

        let output = ParryIntersectGroupQry::query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, pair_skips, &ParryIntersectGroupArgs::new(self.parry_shape_rep.clone(), false));

        let mut a = vec![];

        output.outputs.iter().for_each(|x| {
            if x.data.intersect {
                a.push(x.pair_idxs.clone());
            }
        });

        let selector = convert_parry_pair_idxs_to_parry_pair_selector(a);

        ParryFilterOutput {
            selector,
            duration: start.elapsed(),
            aux_datas: vec![ParryOutputAuxData { num_queries: 0, duration: start.elapsed() }],
        }
    }
}
impl<T: AD, P: O3DPose<T>> OParryPairGroupFilterTrait<T, P> for ParryIntersectGroupFilter { }

pub struct ParryToSubcomponentsFilter { }
impl<T: AD, P: O3DPose<T>> OPairGroupFilterTrait<T, P> for ParryToSubcomponentsFilter {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Output = ParryFilterOutput;

    fn pair_group_filter<S: PairSkipsTrait>(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, _poses_a: &Vec<P>, _poses_b: &Vec<P>, pair_selector: &Self::SelectorType, _pair_skips: &S) -> Self::Output {
        let start = Instant::now();
        let mut subcomponent_idxs = vec![];
        match pair_selector {
            ParryPairSelector::PairsByIdxs(pair_idxs) => {
                pair_idxs.iter().for_each(|x| {
                    match x {
                        ParryPairIdxs::Shapes(x, y) => {
                            let shape_a = &shape_group_a[*x];
                            let shape_b = &shape_group_b[*y];

                            let num_subcomponents_a = shape_a.convex_subcomponents.len();
                            let num_subcomponents_b = shape_b.convex_subcomponents.len();

                            for i in 0..num_subcomponents_a {
                                for j in 0..num_subcomponents_b {
                                    // subcomponent_idxs.push( ( (*x, i), (*y, j) ) );
                                    subcomponent_idxs.push(ParryPairIdxs::ShapeSubcomponents((*x, i), (*y, j)));
                                }
                            }
                        }
                        ParryPairIdxs::ShapeSubcomponents(x, y) => {
                            subcomponent_idxs.push(ParryPairIdxs::ShapeSubcomponents( x.clone(), y.clone() ))
                        }
                    }
                });
            }
            ParryPairSelector::AllPairs => {
                for x in 0..shape_group_a.len() {
                    for y in 0..shape_group_b.len() {
                        let shape_a = &shape_group_a[x];
                        let shape_b = &shape_group_b[y];

                        let num_subcomponents_a = shape_a.convex_subcomponents.len();
                        let num_subcomponents_b = shape_b.convex_subcomponents.len();

                        for i in 0..num_subcomponents_a {
                            for j in 0..num_subcomponents_b {
                                // subcomponent_idxs.push( ( (x, i), (y, j) ) );
                                subcomponent_idxs.push(ParryPairIdxs::ShapeSubcomponents((x, i), (y, j)));
                            }
                        }
                    }
                }
            }
            ParryPairSelector::HalfPairs => {
                for x in 0..shape_group_a.len() {
                    for y in 0..shape_group_b.len() {
                        if x < y {
                            let shape_a = &shape_group_a[x];
                            let shape_b = &shape_group_b[y];

                            let num_subcomponents_a = shape_a.convex_subcomponents.len();
                            let num_subcomponents_b = shape_b.convex_subcomponents.len();

                            for i in 0..num_subcomponents_a {
                                for j in 0..num_subcomponents_b {
                                    // subcomponent_idxs.push(((x, i), (y, j)));
                                    subcomponent_idxs.push(ParryPairIdxs::ShapeSubcomponents((x, i), (y, j)));
                                }
                            }
                        }
                    }
                }
            }
            ParryPairSelector::AllPairsSubcomponents => {
                return self.pair_group_filter(shape_group_a, shape_group_b, _poses_a, _poses_b, &ParryPairSelector::AllPairs, _pair_skips);
            }
            ParryPairSelector::HalfPairsSubcomponents => {
                return self.pair_group_filter(shape_group_a, shape_group_b, _poses_a, _poses_b, &ParryPairSelector::HalfPairs, _pair_skips);
            }
        }
        
        let selector = ParryPairSelector::PairsByIdxs(subcomponent_idxs);
        
        ParryFilterOutput {
            selector,
            duration: start.elapsed(),
            aux_datas: vec![ParryOutputAuxData { num_queries: 0, duration: start.elapsed() }],
        }
    }
}
impl<T: AD, P: O3DPose<T>> OParryPairGroupFilterTrait<T, P> for ParryToSubcomponentsFilter { }

pub struct ParryDistanceGroupSequenceFilter<T: AD> {
    shape_rep_seq: Vec<ParryShapeRep>,
    subcomponent_shape_rep_seq: Vec<ParryShapeRep>,
    distance_threshold: T,
    parry_dis_mode: ParryDisMode
}
impl<T: AD> ParryDistanceGroupSequenceFilter<T> {
    pub fn new(distance_threshold: T, parry_dis_mode: ParryDisMode, shape_rep_seq: Vec<ParryShapeRep>, subcomponent_shape_rep_seq: Vec<ParryShapeRep>) -> Self {
        Self { distance_threshold, parry_dis_mode, shape_rep_seq, subcomponent_shape_rep_seq }
    }
}
impl<T: AD, P: O3DPose<T>> OPairGroupFilterTrait<T, P> for ParryDistanceGroupSequenceFilter<T> {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Output = ParryFilterOutput;

    fn pair_group_filter<S: PairSkipsTrait>(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S) -> Self::Output {
        let start = Instant::now();

        let mut curr = pair_selector.clone();
        let mut aux_datas = vec![];

        self.shape_rep_seq.iter().for_each(|x| {
            let f = ParryDistanceGroupFilter::new(x.clone(), self.parry_dis_mode.clone(), self.distance_threshold);
            let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips);
            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        });

        if self.subcomponent_shape_rep_seq.len() > 0 {
            let f = ParryToSubcomponentsFilter { };
            let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips);
            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        }

        self.subcomponent_shape_rep_seq.iter().for_each(|x| {
            let f = ParryDistanceGroupFilter::new(x.clone(), self.parry_dis_mode.clone(), self.distance_threshold);
            let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips);
            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        });

        ParryFilterOutput {
            selector: curr,
            duration: start.elapsed(),
            aux_datas,
        }
    }
}

pub struct ParryDistanceWrtAverageGroupSequenceFilter<'a, T: AD> {
    shape_rep_seq: Vec<ParryShapeRep>,
    subcomponent_shape_rep_seq: Vec<ParryShapeRep>,
    average_distances: &'a AHashMapWrapper<(u64, u64), T>,
    distance_wrt_average_threshold: T,
    parry_dis_mode: ParryDisMode
}
impl<'a, T: AD> ParryDistanceWrtAverageGroupSequenceFilter<'a, T> {
    pub fn new(shape_rep_seq: Vec<ParryShapeRep>, subcomponent_shape_rep_seq: Vec<ParryShapeRep>, average_distances: &'a AHashMapWrapper<(u64, u64), T>, distance_wrt_average_threshold: T, parry_dis_mode: ParryDisMode) -> Self {
        Self { shape_rep_seq, subcomponent_shape_rep_seq, average_distances, distance_wrt_average_threshold, parry_dis_mode }
    }
}
impl<'a, T: AD, P: O3DPose<T>> OPairGroupFilterTrait<T, P> for ParryDistanceWrtAverageGroupSequenceFilter<'a, T> {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Output = ParryFilterOutput;

    fn pair_group_filter<S: PairSkipsTrait>(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S) -> Self::Output {
        let start = Instant::now();

        let mut curr = pair_selector.clone();
        let mut aux_datas = vec![];

        self.shape_rep_seq.iter().for_each(|x| {
            let f = ParryDistanceWrtAverageGroupFilter::new(x.clone(), self.parry_dis_mode.clone(), self.average_distances, self.distance_wrt_average_threshold);
            let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips);
            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        });

        if self.subcomponent_shape_rep_seq.len() > 0 {
            let f = ParryToSubcomponentsFilter { };
            let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips);
            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        }

        self.subcomponent_shape_rep_seq.iter().for_each(|x| {
            let f = ParryDistanceWrtAverageGroupFilter::new(x.clone(), self.parry_dis_mode.clone(), self.average_distances, self.distance_wrt_average_threshold);
            let res = f.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &curr, pair_skips);
            aux_datas.extend(res.aux_datas);
            curr = res.selector;
        });

        ParryFilterOutput {
            selector: curr,
            duration: start.elapsed(),
            aux_datas,
        }
    }
}

/*
pub struct ParryGenericFilter<T: AD, P: O3DPose<T>> {
    pub (crate) filters: Vec<Box<dyn OParryPairGroupFilterTrait<T, P, ShapeTypeA=OParryShape<T, P>, ShapeTypeB=OParryShape<T, P>, SelectorType=ParryPairSelector, Output=ParryFilterOutput>>>
}
impl<T: AD, P: O3DPose<T>> ParryGenericFilter<T, P> {
    pub fn new(filters: Vec<Box<dyn OParryPairGroupFilterTrait<T, P, ShapeTypeA=OParryShape<T, P>, ShapeTypeB=OParryShape<T, P>, SelectorType=ParryPairSelector, Output=ParryFilterOutput>>>) -> Self {
        assert!(filters.len() > 0);
        Self { filters }
    }
}
impl<T: AD, P: O3DPose<T>> OPairGroupFilterTrait<T, P> for ParryGenericFilter<T, P> {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type SelectorType = ParryPairSelector;
    type Output = ParryFilterOutput;

    fn pair_group_filter(&self, shape_group_a: &Vec<Self::ShapeTypeA>, shape_group_b: &Vec<Self::ShapeTypeB>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType) -> Self::Output {
        let start = Instant::now();
        let selector = pair_selector.clone();
        let mut aux_datas = vec![];
        self.filters.iter().for_each(|x| {
            let res = x.pair_group_filter(shape_group_a, shape_group_b, poses_a, poses_b, &selector);
            res.aux_datas.iter().for_each(|y| aux_datas.push(y.clone()));
        });

        ParryFilterOutput {
            selector,
            duration: start.elapsed(),
            aux_datas
        }
    }
}
*/

pub struct ParryFilterOutput {
    selector: ParryPairSelector,
    duration: Duration,
    aux_datas: Vec<ParryOutputAuxData>
}
impl ParryFilterOutput {
    pub fn selector(&self) -> &ParryPairSelector {
        &self.selector
    }
    pub fn duration(&self) -> Duration {
        self.duration
    }
    pub fn aux_datas(&self) -> &Vec<ParryOutputAuxData> {
        &self.aux_datas
    }
}
impl AsParryFilterOutputTrait for ParryFilterOutput {
    #[inline(always)]
    fn as_parry_filter_output(&self) -> &ParryFilterOutput {
        self
    }
}

#[inline(always)]
pub fn get_parry_ids_from_shape_pair<T: AD, P: O3DPose<T>>(shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep) -> (u64, u64) {
    return match parry_qry_shape_type {
        ParryQryShapeType::Standard => { (shape_a.base_shape.id_from_shape_rep(parry_shape_rep), shape_b.base_shape.id_from_shape_rep(parry_shape_rep)) }
        ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx, shape_b_subcomponent_idx } => {
            (shape_a.convex_subcomponents[*shape_a_subcomponent_idx].id_from_shape_rep(parry_shape_rep), shape_b.convex_subcomponents[*shape_b_subcomponent_idx].id_from_shape_rep(parry_shape_rep))
        }
    }
}

#[inline(always)]
pub fn get_all_parry_pairs_idxs<T: AD, P: O3DPose<T>>(shape_group_a: &Vec<OParryShape<T, P>>, shape_group_b: &Vec<OParryShape<T, P>>, half_pairs: bool, subcomponents: bool) -> Vec<ParryPairIdxs> {
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
                        out.push(ParryPairIdxs::ShapeSubcomponents((i,k), (j,l)));
                    }
                }
            } else {
                out.push(ParryPairIdxs::Shapes(i,j));
            }
        }
    }

    out
}

#[inline]
fn parry_generic_pair_group_query<T: AD, P: O3DPose<T>, S: PairSkipsTrait, O: AsAny, F, Termination>(shape_group_a: &Vec<OParryShape<T, P>>,
                                                                                                     shape_group_b: &Vec<OParryShape<T, P>>,
                                                                                                     poses_a: &Vec<P>,
                                                                                                     poses_b: &Vec<P>,
                                                                                                     pair_selector: &ParryPairSelector,
                                                                                                     parry_shape_rep: &ParryShapeRep,
                                                                                                     pair_skips: &S,
                                                                                                     f: F,
                                                                                                     termination: Termination) -> (Vec<ParryPairGroupOutputWrapper<O>>, usize)
    where F: Fn(&OParryShape<T, P>, &OParryShape<T, P>, &P, &P, &ParryQryShapeType, &ParryShapeRep) -> O,
          Termination: Fn(&O) -> bool
{
    let mut out_vec = vec![];
    let mut count = 0;

    match pair_selector {
        ParryPairSelector::AllPairs => {
            'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                'l2: for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                    let id_a = shape_a.base_shape.id_from_shape_rep(parry_shape_rep);
                    let id_b = shape_b.base_shape.id_from_shape_rep(parry_shape_rep);
                    // let ids = parry_pair_idxs_to_shape_ids(shape_group_a, shape_group_b, &ParryPairIdxs::Shapes(i, j), parry_shape_rep);
                    if pair_skips.skip(id_a, id_b) { continue 'l2; }
                    count += 1;
                    let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::Standard, parry_shape_rep);
                    let terminate = termination(&o);
                    out_vec.push(ParryPairGroupOutputWrapper { data: o, pair_ids: (id_a, id_b), pair_idxs: ParryPairIdxs::Shapes(i, j) });
                    if terminate { break 'l; }
                }
            }
        }
        ParryPairSelector::HalfPairs => {
            'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                'l2: for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                    if i < j {
                        let id_a = shape_a.base_shape.id_from_shape_rep(parry_shape_rep);
                        let id_b = shape_b.base_shape.id_from_shape_rep(parry_shape_rep);
                        if pair_skips.skip(id_a, id_b) { continue 'l2; }
                        count += 1;
                        let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::Standard, parry_shape_rep);
                        let terminate = termination(&o);
                        out_vec.push(ParryPairGroupOutputWrapper { data: o, pair_ids: (id_a,id_b), pair_idxs: ParryPairIdxs::Shapes(i, j) });
                        if terminate { break 'l; }
                    }
                }
            }
        }
        ParryPairSelector::PairsByIdxs(idx_pairs) => {
            'l: for idx_pair in idx_pairs {
                match idx_pair {
                    ParryPairIdxs::Shapes(idx_pair0, idx_pair1) => {
                        let idx0 = *idx_pair0;
                        let idx1 = *idx_pair1;
                        let shape_a = &shape_group_a[idx0];
                        let shape_b = &shape_group_a[idx1];
                        let pose_a = &poses_a[idx0];
                        let pose_b = &poses_b[idx1];

                        let id_a = shape_a.base_shape.id_from_shape_rep(parry_shape_rep);
                        let id_b = shape_b.base_shape.id_from_shape_rep(parry_shape_rep);
                        if pair_skips.skip(id_a, id_b) { continue 'l; }

                        count += 1;
                        let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::Standard, parry_shape_rep);
                        let terminate = termination(&o);
                        out_vec.push(ParryPairGroupOutputWrapper { data: o, pair_ids: (id_a, id_b), pair_idxs: ParryPairIdxs::Shapes(idx0, idx1) });
                        if terminate { break 'l; }
                    }
                    ParryPairIdxs::ShapeSubcomponents(idx_pairs0, idx_pairs1) => {
                        let idxs0 = *idx_pairs0;
                        let idxs1 = *idx_pairs1;
                        let shape_a_idx = idxs0.0;
                        let shape_a_subcomponent_idx = idxs0.1;
                        let shape_b_idx = idxs1.0;
                        let shape_b_subcomponent_idx = idxs1.1;

                        let shape_a = &shape_group_a[shape_a_idx];
                        let shape_b = &shape_group_a[shape_b_idx];
                        let pose_a = &poses_a[shape_a_idx];
                        let pose_b = &poses_b[shape_b_idx];

                        let id_a = shape_a.convex_subcomponents[shape_a_subcomponent_idx].id_from_shape_rep(parry_shape_rep);
                        let id_b = shape_b.convex_subcomponents[shape_b_subcomponent_idx].id_from_shape_rep(parry_shape_rep);
                        if pair_skips.skip(id_a, id_b) { continue 'l; }

                        count += 1;
                        let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx, shape_b_subcomponent_idx }, parry_shape_rep);
                        let terminate = termination(&o);
                        out_vec.push(ParryPairGroupOutputWrapper { data: o, pair_ids: (id_a, id_b), pair_idxs: ParryPairIdxs::ShapeSubcomponents((shape_a_idx, shape_a_subcomponent_idx), (shape_b_idx, shape_b_subcomponent_idx) ) });
                        if terminate { break 'l; }
                    }
                }
            }
        }
        ParryPairSelector::AllPairsSubcomponents => {
            'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                    let num_subcomponents_a = shape_a.convex_subcomponents.len();
                    let num_subcomponents_b = shape_b.convex_subcomponents.len();

                    for k in 0..num_subcomponents_a {
                        let id_a = shape_a.convex_subcomponents.get(k).unwrap().id_from_shape_rep(parry_shape_rep);
                        'l2: for l in 0..num_subcomponents_b {
                            let id_b = shape_b.convex_subcomponents.get(l).unwrap().id_from_shape_rep(parry_shape_rep);
                            if pair_skips.skip(id_a, id_b) { continue 'l2; }
                            count += 1;
                            let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx: k, shape_b_subcomponent_idx: l }, parry_shape_rep);
                            let terminate = termination(&o);
                            out_vec.push(ParryPairGroupOutputWrapper { data: o, pair_ids: (id_a, id_b), pair_idxs: ParryPairIdxs::Shapes(i, j) });
                            if terminate { break 'l; }
                        }
                    }
                }
            }
        }
        ParryPairSelector::HalfPairsSubcomponents => {
            'l: for (i, (shape_a, pose_a)) in shape_group_a.iter().zip(poses_a.iter()).enumerate() {
                for (j, (shape_b, pose_b)) in shape_group_b.iter().zip(poses_b.iter()).enumerate() {
                    if i < j {
                        let num_subcomponents_a = shape_a.convex_subcomponents.len();
                        let num_subcomponents_b = shape_b.convex_subcomponents.len();

                        for k in 0..num_subcomponents_a {
                            let id_a = shape_a.convex_subcomponents.get(k).unwrap().id_from_shape_rep(parry_shape_rep);
                            'l2: for l in 0..num_subcomponents_b {
                                let id_b = shape_b.convex_subcomponents.get(l).unwrap().id_from_shape_rep(parry_shape_rep);
                                if pair_skips.skip(id_a, id_b) { continue 'l2; }
                                count += 1;
                                let o = f(shape_a, shape_b, pose_a, pose_b, &ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx: k, shape_b_subcomponent_idx: l }, parry_shape_rep);
                                let terminate = termination(&o);
                                out_vec.push(ParryPairGroupOutputWrapper { data: o, pair_ids: (id_a, id_b), pair_idxs: ParryPairIdxs::Shapes(i, j) });
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

#[inline]
fn convert_parry_pair_idxs_to_parry_pair_selector(parry_pair_idxs: Vec<ParryPairIdxs>) -> ParryPairSelector {
    let mut a = vec![];

    parry_pair_idxs.iter().for_each(|x| {
        a.push(x.clone());
    });

    ParryPairSelector::PairsByIdxs(a)
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
