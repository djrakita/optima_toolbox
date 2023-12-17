use std::cmp::Ordering;
use std::time::{Duration, Instant};
use ad_trait::AD;
use as_any::AsAny;
use parry_ad::query::Contact;
use serde::{Deserialize, Serialize};
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_3d_spatial::optima_3d_rotation::{O3DRotation};
use optima_3d_spatial::optima_3d_vec::O3DVec;
use crate::shape_queries::{ContactOutputTrait, DistanceBoundsOutputTrait, DistanceLowerBoundOutputTrait, DistanceOutputTrait, DistanceUpperBoundOutputTrait, IntersectOutputTrait, OShpQryContactTrait, OShpQryDistanceTrait, OShpQryIntersectTrait};
use crate::shapes::{OParryShape, OParryShpGeneric};

pub trait OPairQryTrait<T: AD, P: O3DPose<T>> {
    type ShapeTypeA : AsAny;
    type ShapeTypeB : AsAny;
    type Args<'a>;
    type Output : AsAny + PartialOrd;
    fn query<'a>(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args<'a>) -> Self::Output;
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
    type Args<'a> = (ParryQryShapeType, ParryShapeRep);
    type Output = ParryIntersectOutput;

    fn query<'a>(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args<'a>) -> Self::Output {
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
    type Args<'a> = (ParryDisMode, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryDistanceOutput<T>;

    fn query<'a>(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args<'a>) -> Self::Output {
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
    type Args<'a> = (T, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryContactOutput<T>;

    fn query<'a>(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args<'a>) -> Self::Output {
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
    type Args<'a> = (ParryDisMode, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryDistanceLowerBoundOutput<T>;

    fn query<'a>(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args<'a>) -> Self::Output {
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
    type Args<'a> = (ParryDisMode, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryDistanceUpperBoundOutput<T>;

    fn query<'a>(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args<'a>) -> Self::Output {
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
    type Args<'a> = (ParryDisMode, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryDistanceBoundsOutput<T>;

    fn query<'a>(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args<'a>) -> Self::Output {
        parry_shape_lower_and_upper_bound(shape_a, shape_b, pose_a, pose_b, &args.0, &args.1, &args.2, args.3)
    }
}

pub struct ParryProximaDistanceUpperBoundQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryProximaDistanceUpperBoundQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args<'a> = ParryProximaDistanceUpperBoundArgs<'a, T, P, <P::RotationType as O3DRotation<T>>::Native3DVecType>;
    type Output = ParryProximaDistanceUpperBoundOutput<T>;

    fn query<'a>(_shape_a: &Self::ShapeTypeA, _shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args<'a>) -> Self::Output {
        let start = Instant::now();
        // let shapes = get_shapes_from_parry_qry_shape_type_and_parry_shape_rep(shape_a, shape_b, &args.0, &args.1);

        let pose_a_j = args.pose_a_j;
        let pose_b_j = args.pose_b_j;

        let c_a_j = args.closest_point_a_j;
        let c_b_j = args.closest_point_b_j;

        let r_a_j = pose_a_j.rotation();
        let r_b_j = pose_b_j.rotation();

        let r_a_k = pose_a.rotation();
        let r_b_k = pose_b.rotation();

        let t_a_j = pose_a_j.translation();
        let t_b_j = pose_b_j.translation();

        let t_a_k = pose_a.translation();
        let t_b_k = pose_b.translation();

        let ca1 = c_a_j.o3dvec_sub(t_a_j);
        let ca2 = r_a_j.inverse().mul_by_point_native(&ca1);
        let ca3 = r_a_k.mul_by_point_native(&ca2);
        let ca4 = ca3.o3dvec_add(t_a_k);

        let cb1 = c_b_j.o3dvec_sub(t_b_j);
        let cb2 = r_b_j.inverse().mul_by_point_native(&cb1);
        let cb3 = r_b_k.mul_by_point_native(&cb2);
        let cb4 = cb3.o3dvec_add(t_b_k);

        let c = ca4.o3dvec_sub(&cb4);
        let upper_bound = c.norm();
        let upper_bound_wrt_average = match &args.average_distance {
            None => {
                upper_bound
            }
            Some(average) => { upper_bound / *average }
        };

        ParryProximaDistanceUpperBoundOutput {
            distance_upper_bound_wrt_average: upper_bound_wrt_average,
            raw_distance_upper_bound: upper_bound,
            aux_data: ParryOutputAuxData { num_queries: 0, duration: start.elapsed() },
        }
    }
}
pub struct ParryProximaDistanceUpperBoundArgs<'a, T: AD, P: O3DPose<T>, V: O3DVec<T>> {
    pub parry_qry_shape_type: ParryQryShapeType,
    pub parry_shape_rep: ParryShapeRep,
    pub pose_a_j: &'a P,
    pub pose_b_j: &'a P,
    pub closest_point_a_j: &'a V,
    pub closest_point_b_j: &'a V,
    pub average_distance: Option<T>
}

pub struct ParryProximaDistanceLowerBoundQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryProximaDistanceLowerBoundQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args<'a> = ParryProximaDistanceLowerBoundArgs<'a, T, P>;
    type Output = ParryProximaDistanceLowerBoundOutput<T, P>;

    fn query<'a>(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args<'a>) -> Self::Output {
        let start = Instant::now();
        let shapes = get_shapes_from_parry_qry_shape_type_and_parry_shape_rep(shape_a, shape_b, &args.parry_qry_shape_type, &args.parry_shape_rep);
        let a_f = shapes.0.max_dis_from_origin_to_point_on_shape;
        let b_f = shapes.1.max_dis_from_origin_to_point_on_shape;
        let max_f = a_f.max(b_f);
        let max_f2 = T::constant(2.0) * max_f * max_f;

        let displacement_between_a_and_b_k = pose_a.displacement(pose_b);
        let disp_of_disp = args.displacement_between_a_and_b_j.displacement(&displacement_between_a_and_b_k);

        let delta_m = disp_of_disp.translation().norm();
        let delta_r = disp_of_disp.rotation().angle();

        let psi = (max_f2 * (T::one() - delta_r.cos())).sqrt();

        let lower_bound = args.raw_distance_j - delta_m - psi;
        let lower_bound_wrt_average = match &args.average_distance {
            None => { lower_bound }
            Some(average) => { lower_bound / *average }
        };

        ParryProximaDistanceLowerBoundOutput {
            distance_lower_bound_wrt_average: lower_bound_wrt_average,
            raw_distance_lower_bound: lower_bound,
            displacement_between_a_and_b_k,
            aux_data: ParryOutputAuxData { num_queries: 0, duration: start.elapsed() },
        }
    }
}

pub struct ParryProximaDistanceLowerBoundArgs<'a, T: AD, P: O3DPose<T>> {
    pub parry_qry_shape_type: ParryQryShapeType,
    pub parry_shape_rep: ParryShapeRep,
    pub raw_distance_j: T,
    pub displacement_between_a_and_b_j: &'a P,
    pub average_distance: Option<T>
}

pub struct ParryProximaDistanceBoundsQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryProximaDistanceBoundsQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args<'a> = ParryProximaDistanceBoundsArgs<'a, T, P, <P::RotationType as O3DRotation<T>>::Native3DVecType>;
    type Output = ParryProximaDistanceBoundsOutputOption<T, P>;

    fn query<'a>(shape_a: &Self::ShapeTypeA, shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args<'a>) -> Self::Output {
        let start = Instant::now();

        let lower_bound_res = ParryProximaDistanceLowerBoundQry::query(shape_a, shape_b, pose_a, pose_b, &ParryProximaDistanceLowerBoundArgs {
            parry_qry_shape_type: args.parry_qry_shape_type.clone(),
            parry_shape_rep: args.parry_shape_rep.clone(),
            raw_distance_j: args.raw_distance_j,
            displacement_between_a_and_b_j: &args.displacement_between_a_and_b_j,
            average_distance: args.average_distance.clone(),
        });

        // println!("{:?}, {:?}", upper_bound_res.distance_upper_bound(), lower_bound_res.distance_lower_bound_wrt_average);

        if lower_bound_res.distance_lower_bound() > args.cutoff_distance {
            return ParryProximaDistanceBoundsOutputOption(None);
        }

        let upper_bound_res = ParryProximaDistanceUpperBoundQry::query(shape_a, shape_b, pose_a, pose_b, &ParryProximaDistanceUpperBoundArgs {
            parry_qry_shape_type: args.parry_qry_shape_type.clone(),
            parry_shape_rep: args.parry_shape_rep.clone(),
            pose_a_j: args.pose_a_j,
            pose_b_j: args.pose_b_j,
            closest_point_a_j: args.closest_point_a_j,
            closest_point_b_j: args.closest_point_b_j,
            average_distance: args.average_distance.clone(),
        });

        // println!("{:?}, {:?}, {:?}, {:?}, {:?}", lower_bound_res.distance_lower_bound_wrt_average, upper_bound_res.distance_upper_bound_wrt_average, lower_bound_res.raw_distance_lower_bound, upper_bound_res.raw_distance_upper_bound, lower_bound_res.displacement_between_a_and_b_k);

        ParryProximaDistanceBoundsOutputOption(
            Some(ParryProximaDistanceBoundsOutput {
                    distance_lower_bound_wrt_average: lower_bound_res.distance_lower_bound_wrt_average,
                    distance_upper_bound_wrt_average: upper_bound_res.distance_upper_bound_wrt_average,
                    raw_distance_lower_bound: lower_bound_res.raw_distance_lower_bound,
                    raw_distance_upper_bound: upper_bound_res.raw_distance_upper_bound,
                    displacement_between_a_and_b_k: lower_bound_res.displacement_between_a_and_b_k,
                    aux_data: ParryOutputAuxData { num_queries: 0, duration: start.elapsed() },
                })
        )
    }
}

pub struct ParryProximaDistanceBoundsArgs<'a, T: AD, P: O3DPose<T>, V: O3DVec<T>> {
    pub parry_qry_shape_type: ParryQryShapeType,
    pub parry_shape_rep: ParryShapeRep,
    pub pose_a_j: &'a P,
    pub pose_b_j: &'a P,
    pub closest_point_a_j: &'a V,
    pub closest_point_b_j: &'a V,
    pub raw_distance_j: T,
    pub displacement_between_a_and_b_j: &'a P,
    pub cutoff_distance: T,
    pub average_distance: Option<T>
}

/*
pub struct ParryProxima2DistanceBoundsQry;
impl<T: AD, P: O3DPose<T>> OPairQryTrait<T, P> for ParryProxima2DistanceBoundsQry {
    type ShapeTypeA = OParryShape<T, P>;
    type ShapeTypeB = OParryShape<T, P>;
    type Args<'a> = ParryProxima2DistanceBoundsArgs<'a, T, P>;
    type Output = ParryProximaDistanceBoundsOutput<T, P>;

    fn query<'a>(_shape_a: &Self::ShapeTypeA, _shape_b: &Self::ShapeTypeB, pose_a: &P, pose_b: &P, args: &Self::Args<'a>) -> Self::Output {
        let start = Instant::now();
        let displacement_between_a_and_b_k = pose_a.displacement(pose_b);
        let disp_of_disps = args.displacement_between_a_and_b_j.displacement(&displacement_between_a_and_b_k);
        let m = disp_of_disps.magnitude();
        let s = m*args.max_delta_distance_per_unit;
        let lower_bound_distance = args.raw_distance_j - s;
        let upper_bound_distance = args.raw_distance_j + s;

        let average = args.average_distance.unwrap_or_else(|| T::one());

        ParryProximaDistanceBoundsOutput {
            distance_lower_bound_wrt_average: lower_bound_distance / average,
            distance_upper_bound_wrt_average: upper_bound_distance / average,
            raw_distance_lower_bound: lower_bound_distance,
            raw_distance_upper_bound: upper_bound_distance,
            displacement_between_a_and_b_k,
            aux_data: ParryOutputAuxData { num_queries: 0, duration: start.elapsed() },
        }
    }
}

pub struct ParryProxima2DistanceBoundsArgs<'a, T: AD, P: O3DPose<T>> {
    pub parry_qry_shape_type: ParryQryShapeType,
    pub parry_shape_rep: ParryShapeRep,
    pub raw_distance_j: T,
    pub displacement_between_a_and_b_j: &'a P,
    pub max_delta_distance_per_unit: T,
    pub cutoff_distance: T,
    pub average_distance: Option<T>
}
*/

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

#[inline(always)]
pub (crate) fn get_shapes_from_parry_qry_shape_type_and_parry_shape_rep<'a, T: AD, P: O3DPose<T>>(shape_a: &'a OParryShape<T, P>, shape_b: &'a OParryShape<T, P>, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep) -> (&'a OParryShpGeneric<T, P>, &'a OParryShpGeneric<T, P>) {
    let shapes = match parry_qry_shape_type {
        ParryQryShapeType::Standard => {
            match parry_shape_rep {
                ParryShapeRep::Full => { (&shape_a.base_shape.base_shape, &shape_b.base_shape.base_shape) }
                ParryShapeRep::OBB => { (&shape_a.base_shape.obb, &shape_b.base_shape.obb) }
                ParryShapeRep::BoundingSphere => { (&shape_a.base_shape.bounding_sphere, &shape_b.base_shape.bounding_sphere) }
            }
        }
        ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx, shape_b_subcomponent_idx } => {
            match parry_shape_rep {
                ParryShapeRep::Full => { (&shape_a.convex_subcomponents[*shape_a_subcomponent_idx].base_shape, &shape_b.convex_subcomponents[*shape_b_subcomponent_idx].base_shape) }
                ParryShapeRep::OBB => { (&shape_a.convex_subcomponents[*shape_a_subcomponent_idx].base_shape, &shape_b.convex_subcomponents[*shape_b_subcomponent_idx].base_shape) }
                ParryShapeRep::BoundingSphere => { (&shape_a.convex_subcomponents[*shape_a_subcomponent_idx].base_shape, &shape_b.convex_subcomponents[*shape_b_subcomponent_idx].base_shape) }
            }
        }
    };
    shapes
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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ParryDisMode {
    StandardDis, ContactDis
}

#[derive(Clone, Debug)]
pub enum ParryQryShapeType {
    Standard, ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx: usize, shape_b_subcomponent_idx: usize }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
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

#[derive(Clone, Debug)]
pub struct ParryProximaDistanceLowerBoundOutput<T: AD, P: O3DPose<T>> {
    pub (crate) distance_lower_bound_wrt_average: T,
    pub (crate) raw_distance_lower_bound: T,
    pub (crate) displacement_between_a_and_b_k: P,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD, P: O3DPose<T>> ParryProximaDistanceLowerBoundOutput<T, P> {
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
    #[inline(always)]
    pub fn raw_distance_lower_bound(&self) -> &T {
        &self.raw_distance_lower_bound
    }
}
impl<T: AD, P: O3DPose<T>> PartialEq for ParryProximaDistanceLowerBoundOutput<T, P> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.distance_lower_bound_wrt_average.eq(&other.distance_lower_bound_wrt_average)
    }
}
impl<T: AD, P: O3DPose<T>> PartialOrd for ParryProximaDistanceLowerBoundOutput<T, P> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.distance_lower_bound_wrt_average.partial_cmp(&other.distance_lower_bound_wrt_average)
    }
}
impl<T: AD, P: O3DPose<T>> DistanceLowerBoundOutputTrait<T> for ParryProximaDistanceLowerBoundOutput<T, P> {
    #[inline(always)]
    fn distance_lower_bound(&self) -> T {
        self.distance_lower_bound_wrt_average
    }
}


pub struct ParryProximaDistanceUpperBoundOutput<T: AD> {
    pub (crate) distance_upper_bound_wrt_average: T,
    pub (crate) raw_distance_upper_bound: T,
    // pub (crate) shape_ids: (u64, u64),
    // pub (crate) shape_idxs: ParryPairIdxs,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD> ParryProximaDistanceUpperBoundOutput<T> {
    #[inline(always)]
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl<T: AD> PartialEq for ParryProximaDistanceUpperBoundOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.distance_upper_bound_wrt_average.eq(&other.distance_upper_bound_wrt_average)
    }
}
impl<T: AD> PartialOrd for ParryProximaDistanceUpperBoundOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.distance_upper_bound_wrt_average.partial_cmp(&other.distance_upper_bound_wrt_average)
    }
}
impl<T: AD> DistanceUpperBoundOutputTrait<T> for ParryProximaDistanceUpperBoundOutput<T> {
    #[inline(always)]
    fn distance_upper_bound(&self) -> T {
        self.distance_upper_bound_wrt_average
    }
}

#[derive(Clone, Debug)]
#[allow(dead_code)]
pub struct ParryProximaDistanceBoundsOutput<T: AD, P: O3DPose<T>> {
    pub (crate) distance_lower_bound_wrt_average: T,
    pub (crate) distance_upper_bound_wrt_average: T,
    pub (crate) raw_distance_lower_bound: T,
    pub (crate) raw_distance_upper_bound: T,
    pub (crate) displacement_between_a_and_b_k: P,
    pub (crate) aux_data: ParryOutputAuxData
}
impl<T: AD, P: O3DPose<T>> ParryProximaDistanceBoundsOutput<T, P> {
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl<T: AD, P: O3DPose<T>> PartialEq for ParryProximaDistanceBoundsOutput<T, P> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        let self_diff = self.distance_upper_bound_wrt_average - self.distance_lower_bound_wrt_average;
        let other_diff = other.distance_upper_bound_wrt_average - other.distance_lower_bound_wrt_average;

        self_diff.eq(&other_diff)
    }
}
impl<T: AD, P: O3DPose<T>> PartialOrd for ParryProximaDistanceBoundsOutput<T, P> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        let self_diff = self.distance_upper_bound_wrt_average - self.distance_lower_bound_wrt_average;
        let other_diff = other.distance_upper_bound_wrt_average - other.distance_lower_bound_wrt_average;

        other_diff.partial_cmp(&self_diff)
    }
}
impl<T: AD, P: O3DPose<T>> DistanceBoundsOutputTrait<T> for ParryProximaDistanceBoundsOutput<T, P> {
    #[inline(always)]
    fn distance_lower_bound(&self) -> T {
        self.distance_lower_bound_wrt_average
    }

    #[inline(always)]
    fn distance_upper_bound(&self) -> T {
        self.distance_upper_bound_wrt_average
    }
}

pub struct ParryProximaDistanceBoundsOutputOption<T: AD, P: O3DPose<T>>(pub Option<ParryProximaDistanceBoundsOutput<T, P>>);
impl<T: AD, P: O3DPose<T>> PartialEq for ParryProximaDistanceBoundsOutputOption<T, P> {
    fn eq(&self, other: &Self) -> bool {
        match (&self.0, &other.0) {
            (Some(a), Some(b)) => { a.eq(&b) }
            (Some(_), None) => { false }
            (None, Some(_)) => { false }
            (None, None) => { true }
        }
    }
}
impl<T: AD, P: O3DPose<T>> PartialOrd for ParryProximaDistanceBoundsOutputOption<T, P> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match (&self.0, &other.0) {
            (Some(a), Some(b)) => { a.partial_cmp(&b) }
            (Some(_), None) => { Some(Ordering::Less) }
            (None, Some(_)) => { Some(Ordering::Greater) }
            (None, None) => { Some(Ordering::Equal) }
        }
    }
}

