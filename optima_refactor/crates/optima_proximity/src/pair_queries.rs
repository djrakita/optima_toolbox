use std::cmp::Ordering;
use std::time::{Duration, Instant};
use ad_trait::AD;
use as_any::AsAny;
use parry_ad::query::Contact;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use crate::shape_queries::{ContactOutputTrait, DistanceBoundsOutputTrait, DistanceLowerBoundOutputTrait, DistanceOutputTrait, DistanceUpperBoundOutputTrait, IntersectOutputTrait, OShpQryContactTrait, OShpQryDistanceTrait, OShpQryIntersectTrait};
use crate::shapes::{OParryShape};

pub trait OPairQryTrait<T: AD, P: O3DPose<T>> {
    type ShapeTypeA : AsAny;
    type ShapeTypeB : AsAny;
    type Args : AsAny;
    type Output : AsAny + PartialOrd;
    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output;
}

/*
pub trait OPairQryIntersectTrait<T: AD, P: O3DPose<T>> : OPairQryTrait<T, P, Output : IntersectOutputTrait> where Self::ShapeTypeA : OShpQryIntersectTrait<T, P, Self::ShapeTypeB> { }
pub trait OPairQryDistanceTrait<T: AD, P: O3DPose<T>> : OPairQryTrait<T, P, Output : DistanceOutputTrait<T>> where Self::ShapeTypeA : OShpQryIntersectTrait<T, P, Self::ShapeTypeB> { }
pub trait OPairQryContactTrait<T: AD, P: O3DPose<T>> : OPairQryTrait<T, P, Output : ContactOutputTrait<T>> where Self::ShapeTypeA : OShpQryIntersectTrait<T, P, Self::ShapeTypeB> { }
*/

pub struct ParryIntersectQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryIntersectQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryQryShapeType, ParryShapeRep);
    type Output = ParryIntersectOutput;

    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        shape_a.intersect(shape_b, pose_a, pose_b, args)
    }
}
// impl<T: AD, P: O3DPose<T>> OPairQryIntersectTrait<T, P> for ParryIntersectQry { }
#[derive(Clone, Debug)]
pub struct ParryIntersectOutput {
    pub (crate) intersect: bool,
    pub (crate) aux_data: ParryOutputAuxData
}
impl ParryIntersectOutput {
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl PartialEq for ParryIntersectOutput {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.intersect.eq(&other.intersect)
    }
}
impl PartialOrd for ParryIntersectOutput {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        if self.intersect.eq(&other.intersect) { Some(Ordering::Equal) }
        else if self.intersect && !other.intersect { Some(Ordering::Less) }
        else { Some(Ordering::Greater) }
    }
}
impl IntersectOutputTrait for ParryIntersectOutput {
    #[inline(always)]
    fn intersect(&self) -> bool {
        self.intersect
    }
}

pub struct ParryDistanceQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryDistanceOutput<T>;

    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        shape_a.distance(shape_b, pose_a, pose_b, args)
    }
}
// impl<T: AD, P: O3DPose<T>> OPairQryDistanceTrait<T, P> for ParryDistanceQry { }

/*
pub struct ParryDistanceWrtAverageQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceWrtAverageQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep, T);
    type Output = ParryDistanceWrtAverageOutput<T>;

    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let distance = shape_a.distance(shape_b, pose_a, pose_b, &(args.0.clone(), args.1.clone(), args.2.clone()));
        ParryDistanceWrtAverageOutput {
            distance_wrt_average: distance.raw_distance / args.3,
            raw_distance: distance.raw_distance,
            aux_data: distance.aux_data,
        }
    }
}
*/

#[derive(Clone, Debug)]
pub struct ParryContactQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryContactQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (T, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryContactOutput<T>;

    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        shape_a.contact(shape_b, pose_a, pose_b, &args)
    }
}
// impl<T: AD, P: O3DPose<T>> OPairQryContactTrait<T, P> for ParryContactQry { }

/*
pub struct ParryContactWrtAverageQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryContactWrtAverageQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (T, ParryQryShapeType, ParryShapeRep, T);
    type Output = ParryContactWrtAverageOutput<T>;

    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let contact = shape_a.contact(shape_b, pose_a, pose_b, &(args.0.clone(), args.1.clone(), args.2.clone()));
        ParryContactWrtAverageOutput {
            distance_wrt_average: match &contact.contact {
                None => { None }
                Some(c) => { Some(c.dist / args.3) }
            },
            contact: contact.contact,
            aux_data: contact.aux_data,
        }
    }
}
*/

pub struct ParryDistanceLowerBoundQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceLowerBoundQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryDistanceLowerBoundOutput<T>;

    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let res = parry_shape_lower_and_upper_bound(shape_a, shape_b, pose_a, pose_b, &args.0, &args.1, &args.2, args.3);
        ParryDistanceLowerBoundOutput {
            distance_lower_bound_wrt_average: res.distance_lower_bound_wrt_average,
            raw_distance_lower_bound: res.raw_distance_lower_bound,
            aux_data: res.aux_data.clone()
        }
    }
}

/*
pub struct ParryDistanceLowerBoundWrtAverageQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceLowerBoundWrtAverageQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep, T, Option<T>);
    type Output = ParryDistanceLowerBoundWrtAverageOutput<T>;

    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let res = parry_shape_lower_and_upper_bound(shape_a, shape_b, pose_a, pose_b, &args.0, &args.1, &args.2, args.4);
        ParryDistanceLowerBoundWrtAverageOutput {
            distance_lower_bound_wrt_average: res.raw_distance_lower_bound / args.3,
            raw_distance_lower_bound: res.raw_distance_lower_bound,
            aux_data: res.aux_data,
        }
    }
}
*/

pub struct ParryDistanceUpperBoundQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceUpperBoundQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryDistanceUpperBoundOutput<T>;

    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let res = parry_shape_lower_and_upper_bound(shape_a, shape_b, pose_a, pose_b, &args.0, &args.1, &args.2, args.3);
        ParryDistanceUpperBoundOutput {
            distance_upper_bound_wrt_average: res.distance_upper_bound_wrt_average,
            raw_distance_upper_bound: res.raw_distance_upper_bound,
            aux_data: res.aux_data.clone()
        }
    }
}

/*
pub struct ParryDistanceUpperBoundWrtAverageQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceUpperBoundWrtAverageQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep, T);
    type Output = ParryDistanceUpperBoundWrtAverageOutput<T>;

    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let res = parry_shape_lower_and_upper_bound(shape_a, shape_b, pose_a, pose_b, &args.0, &args.1, &args.2);
        ParryDistanceUpperBoundWrtAverageOutput {
            distance_upper_bound_wrt_average: res.raw_distance_upper_bound / args.3,
            raw_distance_upper_bound: res.raw_distance_upper_bound,
            aux_data: res.aux_data,
        }
    }
}
*/

pub struct ParryDistanceBoundsQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceBoundsQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryDistanceBoundsOutput<T>;

    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        parry_shape_lower_and_upper_bound(shape_a, shape_b, pose_a, pose_b, &args.0, &args.1, &args.2, args.3)
    }
}

/*
pub struct ParryDistanceBoundsWrtAverageQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryDistanceBoundsWrtAverageQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep, T);
    type Output = ParryDistanceBoundsWrtAverageOutput<T>;

    fn query(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let res = parry_shape_lower_and_upper_bound(shape_a, shape_b, pose_a, pose_b, &args.0, &args.1, &args.2);
        ParryDistanceBoundsWrtAverageOutput {
            distance_lower_bound_wrt_average: res.raw_distance_lower_bound / args.3,
            distance_upper_bound_wrt_average: res.raw_distance_upper_bound / args.3,
            raw_distance_lower_bound: res.raw_distance_lower_bound,
            raw_distance_upper_bound: res.raw_distance_upper_bound,
            aux_data: res.aux_data,
        }
    }
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

pub (crate) fn parry_shape_lower_and_upper_bound<T: AD, P: O3DPose<T>>(shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_dis_mode: &ParryDisMode, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep, average_dis: Option<T>) -> ParryDistanceBoundsOutput<T> {
    let start = Instant::now();
    let average_dis = match average_dis {
        None => { T::one() }
        Some(a) => {a}
    };

    return match parry_qry_shape_type {
        ParryQryShapeType::Standard => {
            let dis = shape_b.base_shape().distance(shape_b.base_shape(), pose_a, pose_b, &(parry_dis_mode.clone(), parry_shape_rep.clone(), Some(average_dis)));
            let upper_bound = match parry_shape_rep {
                ParryShapeRep::Full => { dis.raw_distance }
                ParryShapeRep::OBB => { dis.raw_distance  + shape_a.base_shape.obb_max_dis_error + shape_b.base_shape.obb_max_dis_error }
                ParryShapeRep::BoundingSphere => { dis.raw_distance  + shape_a.base_shape.bounding_sphere_max_dis_error + shape_b.base_shape.bounding_sphere_max_dis_error }
            };
            ParryDistanceBoundsOutput {
                distance_lower_bound_wrt_average: dis.raw_distance / average_dis,
                distance_upper_bound_wrt_average: upper_bound/average_dis,
                raw_distance_lower_bound: dis.raw_distance,
                raw_distance_upper_bound: upper_bound,
                aux_data: ParryOutputAuxData { num_queries: 1, duration: start.elapsed() },
            }
        }
        /*
        ParryQryShapeType::AllConvexSubcomponents => {
            let mut count = 0;
            let mut lower_bound = T::constant(f64::MAX);
            let mut max_upper_bound = T::constant(f64::MIN);
            for c1 in shape_a.convex_subcomponents.iter() {
                for c2 in shape_b.convex_subcomponents.iter() {
                    count += 1;
                    let dis = c1.distance(&c2, pose_a, pose_b, &(parry_dis_mode.clone(), parry_shape_rep.clone()));
                    let upper_bound = match parry_shape_rep {
                        ParryShapeRep::Full => { dis.distance }
                        ParryShapeRep::OBB => { dis.distance + shape_a.base_shape.obb_max_dis_error + shape_b.base_shape.obb_max_dis_error }
                        ParryShapeRep::BoundingSphere => { dis.distance + shape_a.base_shape.bounding_sphere_max_dis_error + shape_b.base_shape.bounding_sphere_max_dis_error }
                    };
                    // if upper_bound > max_upper_bound { max_upper_bound = upper_bound }
                    if dis.distance < lower_bound {
                        lower_bound = dis.distance;
                        max_upper_bound = upper_bound;
                    }
                }
            }

            ParryDistanceBoundsOutput {
                distance_lower_bound: lower_bound,
                distance_upper_bound: max_upper_bound,
                aux_data: ParryOutputAuxData { num_queries: count, duration: start.elapsed() },
            }
        }
        */
        ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx, shape_b_subcomponent_idx } => {
            let shape_a_ = shape_a.convex_subcomponents.get(*shape_a_subcomponent_idx).expect("idx error");
            let shape_b_ = shape_b.convex_subcomponents.get(*shape_b_subcomponent_idx).expect("idx error");

            let dis = shape_a_.distance(shape_b_, pose_a, pose_b, &(parry_dis_mode.clone(), parry_shape_rep.clone(), Some(average_dis)));
            let upper_bound = match parry_shape_rep {
                ParryShapeRep::Full => { dis.raw_distance }
                ParryShapeRep::OBB => { dis.raw_distance + shape_a.base_shape.obb_max_dis_error + shape_b.base_shape.obb_max_dis_error }
                ParryShapeRep::BoundingSphere => { dis.raw_distance + shape_a.base_shape.bounding_sphere_max_dis_error + shape_b.base_shape.bounding_sphere_max_dis_error }
            };
            ParryDistanceBoundsOutput {
                distance_lower_bound_wrt_average: dis.raw_distance / average_dis,
                distance_upper_bound_wrt_average: upper_bound/average_dis,
                raw_distance_lower_bound: dis.raw_distance,
                raw_distance_upper_bound: upper_bound,
                aux_data: ParryOutputAuxData { num_queries: 1, duration: start.elapsed() },
            }
        }
    };
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug)]
pub struct ParryOutputAuxData {
    pub (crate) num_queries: usize,
    pub (crate) duration: Duration
}
impl ParryOutputAuxData {
    #[inline(always)]
    pub fn num_queries(&self) -> usize {
        self.num_queries
    }
    #[inline(always)]
    pub fn duration(&self) -> Duration {
        self.duration
    }
}

#[derive(Clone, Debug)]
pub enum ParryDisMode {
    StandardDis, ContactDis
}

#[derive(Clone, Debug)]
pub enum ParryQryShapeType {
    Standard, ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx: usize, shape_b_subcomponent_idx: usize }
}

#[derive(Clone, Debug)]
pub enum ParryShapeRep {
    Full, OBB, BoundingSphere
}

#[derive(Clone, Debug)]
pub enum ParryApproximationRep {
    OBB, BoundingSphere
}
impl ParryApproximationRep {
    pub fn to_shape_rep(&self) -> ParryShapeRep {
        match self {
            ParryApproximationRep::OBB => { ParryShapeRep::OBB }
            ParryApproximationRep::BoundingSphere => { ParryShapeRep::BoundingSphere }
        }
    }
}

#[derive(Clone, Debug)]
pub struct ParryDistanceOutput<T: AD> {
    pub (crate) distance_wrt_average: T,
    pub (crate) raw_distance: T,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceOutput<T> {
    #[inline(always)]
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
    #[inline(always)]
    pub fn raw_distance(&self) -> &T {
        &self.raw_distance
    }
}
impl<T: AD> PartialEq for ParryDistanceOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.distance_wrt_average.eq(&other.raw_distance)
    }
}
impl<T: AD> PartialOrd for ParryDistanceOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.distance_wrt_average.partial_cmp(&other.distance_wrt_average)
    }
}
impl<T: AD> DistanceOutputTrait<T> for ParryDistanceOutput<T> {
    #[inline(always)]
    fn distance(&self) -> T {
        self.distance_wrt_average
    }
}

/*
#[derive(Clone, Debug)]
pub struct ParryDistanceWrtAverageOutput<T: AD> {
    pub (crate) distance_wrt_average: T,
    pub (crate) raw_distance: T,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceWrtAverageOutput<T> {
    #[inline(always)]
    pub fn distance_wrt_average(&self) -> T {
        self.distance_wrt_average
    }
    #[inline(always)]
    pub fn raw_distance(&self) -> T {
        self.raw_distance
    }
}
impl<T: AD> ParryDistanceWrtAverageOutput<T> {
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl<T: AD> PartialEq for ParryDistanceWrtAverageOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.distance_wrt_average.eq(&other.distance_wrt_average)
    }
}
impl<T: AD> PartialOrd for ParryDistanceWrtAverageOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.distance_wrt_average.partial_cmp(&other.distance_wrt_average)
    }
}
impl<T: AD> DistanceOutputTrait<T> for ParryDistanceWrtAverageOutput<T> {
    #[inline(always)]
    fn distance(&self) -> T {
        self.distance_wrt_average
    }
}
*/

#[derive(Clone, Debug)]
pub struct ParryContactOutput<T: AD> {
    pub (crate) distance_wrt_average: Option<T>,
    pub (crate) contact: Option<Contact<T>>,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD> ParryContactOutput<T> {
    #[inline(always)]
    pub fn contact(&self) -> Option<Contact<T>> {
        self.contact
    }
    #[inline(always)]
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl<T: AD> PartialEq for ParryContactOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        match (&self.distance_wrt_average, &other.distance_wrt_average) {
            (Some(a), Some(b)) => { a.eq(b) }
            (Some(_), None) => { false }
            (None, Some(_)) => { false }
            (None, None) => { true }
        }
    }
}
impl<T: AD> PartialOrd for ParryContactOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match (&self.distance_wrt_average, &other.distance_wrt_average) {
            (Some(a), Some(b)) => { a.partial_cmp(&b) }
            (Some(_), None) => { Some(Ordering::Less) }
            (None, Some(_)) => { Some(Ordering::Greater) }
            (None, None) => { Some(Ordering::Equal) }
        }
    }
}
impl<T: AD> ContactOutputTrait<T> for ParryContactOutput<T> {
    #[inline(always)]
    fn signed_distance(&self) -> Option<T> {
        self.distance_wrt_average
    }
}

/*
#[derive(Clone, Debug)]
pub struct ParryContactWrtAverageOutput<T: AD> {
    pub (crate) distance_wrt_average: Option<T>,
    pub (crate) contact: Option<Contact<T>>,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD> ParryContactWrtAverageOutput<T> {
    #[inline(always)]
    pub fn contact(&self) -> Option<Contact<T>> {
        self.contact
    }
    #[inline(always)]
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
    #[inline(always)]
    pub fn distance_wrt_average(&self) -> &Option<T> {
        &self.distance_wrt_average
    }
}
impl<T: AD> PartialEq for ParryContactWrtAverageOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        match (&self.distance_wrt_average, &other.distance_wrt_average) {
            (Some(a), Some(b)) => { a.eq(&b) }
            (Some(_), None) => { false }
            (None, Some(_)) => { false }
            (None, None) => { true }
        }
    }
}
impl<T: AD> PartialOrd for ParryContactWrtAverageOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match (&self.distance_wrt_average, &other.distance_wrt_average) {
            (Some(a), Some(b)) => { a.partial_cmp(&b) }
            (Some(_), None) => { Some(Ordering::Less) }
            (None, Some(_)) => { Some(Ordering::Greater) }
            (None, None) => { Some(Ordering::Equal) }
        }
    }
}
impl<T: AD> ContactOutputTrait<T> for ParryContactWrtAverageOutput<T> {
    #[inline(always)]
    fn signed_distance(&self) -> Option<T> {
        self.distance_wrt_average
    }
}
*/

#[derive(Clone, Debug)]
pub struct ParryDistanceLowerBoundOutput<T: AD> {
    pub (crate) distance_lower_bound_wrt_average: T,
    pub (crate) raw_distance_lower_bound: T,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceLowerBoundOutput<T> {
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
    #[inline(always)]
    pub fn raw_distance_lower_bound(&self) -> &T {
        &self.raw_distance_lower_bound
    }
}
impl<T: AD> PartialEq for ParryDistanceLowerBoundOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.distance_lower_bound_wrt_average.eq(&other.distance_lower_bound_wrt_average)
    }
}
impl<T: AD> PartialOrd for ParryDistanceLowerBoundOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.distance_lower_bound_wrt_average.partial_cmp(&other.distance_lower_bound_wrt_average)
    }
}
impl<T: AD> DistanceLowerBoundOutputTrait<T> for ParryDistanceLowerBoundOutput<T> {
    #[inline(always)]
    fn distance_lower_bound(&self) -> T {
        self.distance_lower_bound_wrt_average
    }
}

/*
#[derive(Clone, Debug)]
pub struct ParryDistanceLowerBoundWrtAverageOutput<T: AD> {
    pub (crate) distance_lower_bound_wrt_average: T,
    pub (crate) raw_distance_lower_bound: T,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceLowerBoundWrtAverageOutput<T> {
    #[inline(always)]
    pub fn distance_lower_bound_wrt_average(&self) -> T {
        self.distance_lower_bound_wrt_average
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
    #[inline(always)]
    pub fn raw_distance_lower_bound(&self) -> &T {
        &self.raw_distance_lower_bound
    }
}
impl<T: AD> PartialEq for ParryDistanceLowerBoundWrtAverageOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.distance_lower_bound_wrt_average.eq(&other.distance_lower_bound_wrt_average)
    }
}
impl<T: AD> PartialOrd for ParryDistanceLowerBoundWrtAverageOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.distance_lower_bound_wrt_average.partial_cmp(&other.distance_lower_bound_wrt_average)
    }
}
impl<T: AD> DistanceLowerBoundOutputTrait<T> for ParryDistanceLowerBoundWrtAverageOutput<T> {
    #[inline(always)]
    fn distance_lower_bound(&self) -> T {
        self.distance_lower_bound_wrt_average
    }
}
*/

#[derive(Clone, Debug)]
pub struct ParryDistanceUpperBoundOutput<T: AD> {
    pub (crate) distance_upper_bound_wrt_average: T,
    pub (crate) raw_distance_upper_bound: T,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceUpperBoundOutput<T> {
    #[inline(always)]
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
    #[inline(always)]
    pub fn raw_distance_upper_bound(&self) -> &T {
        &self.raw_distance_upper_bound
    }
}
impl<T: AD> PartialEq for ParryDistanceUpperBoundOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.distance_upper_bound_wrt_average.eq(&other.distance_upper_bound_wrt_average)
    }
}
impl<T: AD> PartialOrd for ParryDistanceUpperBoundOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.distance_upper_bound_wrt_average.partial_cmp(&other.distance_upper_bound_wrt_average)
    }
}
impl<T: AD> DistanceUpperBoundOutputTrait<T> for ParryDistanceUpperBoundOutput<T> {
    #[inline(always)]
    fn distance_upper_bound(&self) -> T {
        self.distance_upper_bound_wrt_average
    }
}

/*
#[derive(Clone, Debug)]
pub struct ParryDistanceUpperBoundWrtAverageOutput<T: AD> {
    pub (crate) distance_upper_bound_wrt_average: T,
    pub (crate) raw_distance_upper_bound: T,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceUpperBoundWrtAverageOutput<T> {
    #[inline(always)]
    pub fn distance_upper_bound_wrt_average(&self) -> T {
        self.distance_upper_bound_wrt_average
    }
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
    #[inline(always)]
    pub fn raw_distance_upper_bound(&self) -> &T {
        &self.raw_distance_upper_bound
    }
}
impl<T: AD> PartialEq for ParryDistanceUpperBoundWrtAverageOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.distance_upper_bound_wrt_average.eq(&other.distance_upper_bound_wrt_average)
    }
}
impl<T: AD> PartialOrd for ParryDistanceUpperBoundWrtAverageOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.distance_upper_bound_wrt_average.partial_cmp(&other.distance_upper_bound_wrt_average)
    }
}
impl<T: AD> DistanceLowerBoundOutputTrait<T> for ParryDistanceUpperBoundWrtAverageOutput<T> {
    #[inline(always)]
    fn distance_lower_bound(&self) -> T {
        self.distance_upper_bound_wrt_average
    }
}
*/

#[derive(Clone, Debug)]
pub struct ParryDistanceBoundsOutput<T: AD> {
    pub (crate) distance_lower_bound_wrt_average: T,
    pub (crate) distance_upper_bound_wrt_average: T,
    pub (crate) raw_distance_lower_bound: T,
    pub (crate) raw_distance_upper_bound: T,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceBoundsOutput<T> {
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl<T: AD> PartialEq for ParryDistanceBoundsOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        let self_diff = self.distance_upper_bound_wrt_average - self.distance_lower_bound_wrt_average;
        let other_diff = other.distance_upper_bound_wrt_average - other.distance_lower_bound_wrt_average;

        self_diff.eq(&other_diff)
    }
}
impl<T: AD> PartialOrd for ParryDistanceBoundsOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        let self_diff = self.distance_upper_bound_wrt_average - self.distance_lower_bound_wrt_average;
        let other_diff = other.distance_upper_bound_wrt_average - other.distance_lower_bound_wrt_average;

        other_diff.partial_cmp(&self_diff)
    }
}
impl<T: AD> DistanceBoundsOutputTrait<T> for ParryDistanceBoundsOutput<T> {
    #[inline(always)]
    fn distance_lower_bound(&self) -> T {
        self.distance_lower_bound_wrt_average
    }

    #[inline(always)]
    fn distance_upper_bound(&self) -> T {
        self.distance_upper_bound_wrt_average
    }
}

/*
#[derive(Clone, Debug)]
pub struct ParryDistanceBoundsWrtAverageOutput<T: AD> {
    pub (crate) distance_lower_bound_wrt_average: T,
    pub (crate) distance_upper_bound_wrt_average: T,
    pub (crate) raw_distance_lower_bound: T,
    pub (crate) raw_distance_upper_bound: T,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceBoundsWrtAverageOutput<T> {
    #[inline(always)]
    pub fn distance_lower_bound_wrt_average(&self) -> T {
        self.distance_lower_bound_wrt_average
    }
    #[inline(always)]
    pub fn distance_upper_bound_wrt_average(&self) -> T {
        self.distance_upper_bound_wrt_average
    }
    #[inline(always)]
    pub fn raw_distance_lower_bound(&self) -> T {
        self.raw_distance_lower_bound
    }
    #[inline(always)]
    pub fn raw_distance_upper_bound(&self) -> T {
        self.raw_distance_upper_bound
    }
}
impl<T: AD> ParryDistanceBoundsWrtAverageOutput<T> {
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl<T: AD> PartialEq for ParryDistanceBoundsWrtAverageOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        let self_diff = self.distance_upper_bound_wrt_average - self.distance_lower_bound_wrt_average;
        let other_diff = other.raw_distance_upper_bound - other.distance_lower_bound_wrt_average;

        self_diff.eq(&other_diff)
    }
}
impl<T: AD> PartialOrd for ParryDistanceBoundsWrtAverageOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        let self_diff = self.distance_upper_bound_wrt_average - self.distance_lower_bound_wrt_average;
        let other_diff = other.raw_distance_upper_bound - other.distance_lower_bound_wrt_average;

        other_diff.partial_cmp(&self_diff)
    }
}
impl<T: AD> DistanceBoundsOutputTrait<T> for ParryDistanceBoundsWrtAverageOutput<T> {
    #[inline(always)]
    fn distance_lower_bound(&self) -> T {
        self.distance_lower_bound_wrt_average
    }

    #[inline(always)]
    fn distance_upper_bound(&self) -> T {
        self.distance_upper_bound_wrt_average
    }
}
*/