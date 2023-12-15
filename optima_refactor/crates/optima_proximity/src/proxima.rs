use std::sync::RwLock;
use std::time::{Duration, Instant};
use ad_trait::AD;
use parry_ad::na::{Isometry3, Vector3};
use serde::{Deserialize, Serialize};
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_3d_spatial::optima_3d_rotation::O3DRotation;
use optima_universal_hashmap::AHashMapWrapper;
use crate::pair_group_queries::{get_parry_ids_from_shape_pair, OPairGroupQryTrait, OwnedPairGroupQry, PairAverageDistanceTrait, PairGroupQryArgsCategory, PairGroupQryOutputCategoryTrait, PairSkipsTrait, parry_generic_pair_group_query, ParryContactGroupArgs, ParryContactGroupQry, ParryPairIdxs, ParryPairSelector, ProximityLossFunction, ToParryProximityOutputTrait};
use crate::pair_queries::{OPairQryTrait, ParryOutputAuxData, ParryProximaDistanceBoundsArgs, ParryProximaDistanceBoundsOutput, ParryProximaDistanceBoundsOutputOption, ParryProximaDistanceBoundsQry, ParryQryShapeType, ParryShapeRep};
use crate::shapes::{OParryShape, ShapeCategoryOParryShape, ShapeCategoryTrait};
use serde_with::serde_as;
use ad_trait::SerdeAD;
use optima_3d_spatial::optima_3d_pose::SerdeO3DPose;
use optima_3d_spatial::optima_3d_vec::{O3DVec, SerdeO3DVec};
use crate::shape_queries::OShpQryContactTrait;

pub struct ParryProximaQry;
impl OPairGroupQryTrait for ParryProximaQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = ParryPairSelector;
    type ArgsCategory = PairGroupQryArgsCategoryParryProxima;
    type OutputCategory = PairGroupQryOutputCategoryParryProxima;

    fn query<'a, T: AD, P: O3DPose<T>, S: PairSkipsTrait, A: PairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, args: &<Self::ArgsCategory as PairGroupQryArgsCategory>::Args<'a, T>) -> <Self::OutputCategory as PairGroupQryOutputCategoryTrait>::Output<T, P> {
        let start = Instant::now();

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryProximaDistanceBoundsOutputOption<T, P> {
            let ids = get_parry_ids_from_shape_pair(shape_a, shape_b, parry_qry_shape_type, parry_shape_rep);
            let binding = args.proxima_container.blocks.read().unwrap();
            let block = binding.hashmap.get(&ids);
            let average_distance = match args.use_average_distance {
                true => {
                    Some(pair_average_distances.average_distance(ids.0, ids.1))
                }
                false => {
                    None
                }
            };

            match block {
                None => {
                    drop(binding);
                    let contact = shape_a.contact(shape_b, pose_a, pose_b, &(T::constant(f64::INFINITY), parry_qry_shape_type.clone(), parry_shape_rep.clone(), None));
                    let contact = contact.contact.unwrap();

                    let disp = pose_a.displacement(&pose_b);
                    let mut binding = args.proxima_container.blocks.write().unwrap();
                    binding.hashmap.insert(ids, ProximaGenericBlock {
                        pose_a_j: pose_a.clone().o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().clone(),
                        pose_b_j: pose_b.clone().o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().clone(),
                        closest_point_a_j: contact.point1.o3dvec_downcast_or_convert::<Vector3<T>>().as_ref().clone(),
                        closest_point_b_j: contact.point2.o3dvec_downcast_or_convert::<Vector3<T>>().as_ref().clone(),
                        raw_distance_j: contact.dist,
                        displacement_between_a_and_b_j: disp.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().clone(),
                    });

                    let output = ParryProximaDistanceBoundsOutputOption {
                        0: Some(ParryProximaDistanceBoundsOutput {
                            distance_lower_bound_wrt_average: match average_distance {
                                None => { contact.dist }
                                Some(a) => { contact.dist / a }
                            },
                            distance_upper_bound_wrt_average: match average_distance {
                                None => { contact.dist }
                                Some(a) => { contact.dist / a }
                            },
                            raw_distance_lower_bound: contact.dist,
                            raw_distance_upper_bound: contact.dist,
                            displacement_between_a_and_b_k: disp.clone(),
                            aux_data: ParryOutputAuxData { num_queries: 1, duration: Default::default() },
                        }),
                    };

                    output
                }
                Some(block) => {
                    let output = ParryProximaDistanceBoundsQry::query(shape_a, shape_b, pose_a, pose_b, &ParryProximaDistanceBoundsArgs {
                        parry_qry_shape_type: parry_qry_shape_type.clone(),
                        parry_shape_rep: parry_shape_rep.clone(),
                        pose_a_j: block.pose_a_j.o3dpose_downcast_or_convert::<P>().as_ref(),
                        pose_b_j: block.pose_b_j.o3dpose_downcast_or_convert::<P>().as_ref(),
                        closest_point_a_j: block.closest_point_a_j.o3dvec_downcast_or_convert::<<P::RotationType as O3DRotation<T>>::Native3DVecType>().as_ref(),
                        closest_point_b_j: block.closest_point_b_j.o3dvec_downcast_or_convert::<<P::RotationType as O3DRotation<T>>::Native3DVecType>().as_ref(),
                        raw_distance_j: block.raw_distance_j,
                        displacement_between_a_and_b_j: block.displacement_between_a_and_b_j.o3dpose_downcast_or_convert::<P>().as_ref(),
                        cutoff_distance: args.cutoff_distance,
                        average_distance,
                    });
                    return output;
                }
            }
        };

        let termination = |_o: &ParryProximaDistanceBoundsOutputOption<T, P>| { false };

        let (outputs, _) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, args.for_filter, f, termination);

        let loss = ProximityLossFunction::Hinge;

        let mut v = vec![];
        let mut proximity_lower_bound_sum = T::zero();
        let mut proximity_upper_bound_sum = T::zero();
        outputs.iter().for_each(|x| {
            match &x.data().0 {
                None => {}
                Some(data) => {
                    let lower_bound_through_loss_and_powf = loss.loss(data.distance_upper_bound_wrt_average, args.cutoff_distance).powf(args.p_norm);
                    let upper_bound_through_loss_and_powf = loss.loss(data.distance_lower_bound_wrt_average, args.cutoff_distance).powf(args.p_norm);
                    proximity_lower_bound_sum += lower_bound_through_loss_and_powf;
                    proximity_upper_bound_sum += upper_bound_through_loss_and_powf;
                    v.push( (lower_bound_through_loss_and_powf, upper_bound_through_loss_and_powf, &data.displacement_between_a_and_b_k, x.pair_ids(), x.pair_idxs()) );
                }
            }
        });

        v.sort_by(|x, y| { (y.1 - y.0).partial_cmp(&(x.1 - x.0)).unwrap() });

        let mut proximity_lower_bound_output;
        let mut proximity_upper_bound_output;
        let mut max_possible_error;

        let mut num_queries = 0;
        
        'l: loop {
            proximity_lower_bound_output = proximity_lower_bound_sum.powf(args.p_norm.recip());
            proximity_upper_bound_output = proximity_upper_bound_sum.powf(args.p_norm.recip());
            assert!(proximity_upper_bound_output >= proximity_lower_bound_output);
            max_possible_error = proximity_upper_bound_output - proximity_lower_bound_output;

            if num_queries >= v.len() { break 'l; }

            let mut terminate = false;
            match &args.termination {
                ProximaTermination::MaxTime(t) => {
                    if start.elapsed() > *t { terminate = true; }
                }
                ProximaTermination::MaxError(e) => {
                    if max_possible_error.to_constant() < * e { terminate = true; }
                }
            }

            if terminate { break 'l; }

            let curr_entry = &v[num_queries];
            let lower_bound_through_loss_and_powf = curr_entry.0;
            let upper_bound_through_loss_and_powf = curr_entry.1;
            let displacement_between_a_and_b_k = curr_entry.2;
            let ids = curr_entry.3;
            let idxs = curr_entry.4;
            let poses = match idxs {
                ParryPairIdxs::Shapes(i, j) => { (&poses_a[*i], &poses_b[*j]) }
                ParryPairIdxs::ShapeSubcomponents((i, _), (j, _)) => { (&poses_a[*i], &poses_b[*j]) }
            };

            let selector = ParryPairSelector::PairsByIdxs(vec![idxs.clone()]);

            let res = ParryContactGroupQry::query(shape_group_a, shape_group_b, poses_a, poses_b, &selector, pair_skips, pair_average_distances, &ParryContactGroupArgs::new(args.parry_shape_rep.clone(), args.cutoff_distance, args.use_average_distance, args.for_filter, T::constant(f64::MIN)));
            let output = &res.outputs()[0];
            let data = output.data();
            let distance_wrt_average = data.distance_wrt_average.expect("this should never be none");
            let contact = data.contact().expect("this should never be none");
            let raw_distance = &contact.dist;

            let ground_truth_through_loss_and_powf = loss.loss(distance_wrt_average, args.cutoff_distance).powf(args.p_norm);
            proximity_lower_bound_sum -= lower_bound_through_loss_and_powf;
            proximity_upper_bound_sum -= upper_bound_through_loss_and_powf;
            proximity_lower_bound_sum += ground_truth_through_loss_and_powf;
            proximity_upper_bound_sum += ground_truth_through_loss_and_powf;

            let mut binding = args.proxima_container.blocks.write().unwrap();
            let block = binding.hashmap.get_mut(&ids);
            match block {
                None => {
                    binding.hashmap.insert(ids, ProximaGenericBlock {
                        pose_a_j: poses.0.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().clone(),
                        pose_b_j: poses.1.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().clone(),
                        closest_point_a_j: contact.point1.o3dvec_downcast_or_convert::<Vector3<T>>().as_ref().clone(),
                        closest_point_b_j: contact.point2.o3dvec_downcast_or_convert::<Vector3<T>>().as_ref().clone(),
                        raw_distance_j: *raw_distance,
                        displacement_between_a_and_b_j: displacement_between_a_and_b_k.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().clone(),
                    });
                }
                Some(block) => {
                    block.displacement_between_a_and_b_j = displacement_between_a_and_b_k.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().clone();
                    block.raw_distance_j = *raw_distance;
                    block.closest_point_a_j = contact.point1.o3dvec_downcast_or_convert::<Vector3<T>>().as_ref().clone();
                    block.closest_point_b_j = contact.point2.o3dvec_downcast_or_convert::<Vector3<T>>().as_ref().clone();
                    block.pose_a_j = poses.0.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().clone();
                    block.pose_b_j = poses.1.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().clone();
                }
            }

            num_queries += 1;
        }

        Box::new(ParryProximaGroupOutput {
            output_proximity_value: proximity_upper_bound_output,
            maximum_possible_proximity_value_error: max_possible_error,
            aux_data: ParryOutputAuxData { num_queries, duration: start.elapsed() },
        })
    }
}

pub type OwnedParryProximaQry<'a, T> = OwnedPairGroupQry<'a, T, ParryProximaQry>;

pub struct PairGroupQryArgsCategoryParryProxima;
impl PairGroupQryArgsCategory for PairGroupQryArgsCategoryParryProxima {
    type Args<'a, T: AD> = PairGroupQryArgsParryProxima<T>;
    type QueryType = ParryProximaQry;
}

pub struct PairGroupQryOutputCategoryParryProxima;
impl PairGroupQryOutputCategoryTrait for PairGroupQryOutputCategoryParryProxima {
    type Output<T: AD, P: O3DPose<T>> = Box<ParryProximaGroupOutput<T>>;
}

pub struct ParryProximaGroupOutput<T: AD> {
    output_proximity_value: T,
    maximum_possible_proximity_value_error: T,
    aux_data: ParryOutputAuxData
}
impl<T: AD> ParryProximaGroupOutput<T> {
    #[inline(always)]
    pub fn output_proximity_value(&self) -> T {
        self.output_proximity_value
    }
    #[inline(always)]
    pub fn maximum_possible_proximity_value_error(&self) -> T {
        self.maximum_possible_proximity_value_error
    }
    #[inline(always)]
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}

impl<T: AD> ToParryProximityOutputTrait<T> for ParryProximaGroupOutput<T> {
    fn get_proximity_objective_value(&self, _cutoff: T, _p_norm: T, _loss_function: ProximityLossFunction) -> T {
        self.output_proximity_value
    }
}

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct PairGroupQryArgsParryProxima<T: AD> {
    #[serde(deserialize_with = "ProximaGenericContainer::<T, Isometry3::<T>>::deserialize")]
    proxima_container: ProximaGenericContainer<T, Isometry3<T>>,
    parry_shape_rep: ParryShapeRep,
    use_average_distance: bool,
    for_filter: bool,
    termination: ProximaTermination,
    #[serde_as(as = "SerdeAD<T>")]
    p_norm: T,
    #[serde_as(as = "SerdeAD<T>")]
    cutoff_distance: T
}
impl<T: AD> PairGroupQryArgsParryProxima<T> {
    pub fn new(parry_shape_rep: ParryShapeRep, use_average_distance: bool, for_filter: bool, termination: ProximaTermination, p_norm: T, cutoff_distance: T) -> Self {
        let proxima_container = ProximaGenericContainer::new();

        Self { proxima_container, parry_shape_rep, use_average_distance, for_filter, termination, p_norm, cutoff_distance }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ProximaTermination {
    MaxTime(Duration),
    MaxError(f64)
}

#[derive(Serialize, Deserialize)]
pub struct ProximaGenericContainer<T: AD, P: O3DPose<T>> {
    #[serde(deserialize_with = "RwLock::<AHashMapWrapper::<(u64, u64), ProximaGenericBlock::<T, P>>>::deserialize")]
    blocks: RwLock<AHashMapWrapper<(u64, u64), ProximaGenericBlock<T, P>>>
}
impl<T: AD, P: O3DPose<T>> ProximaGenericContainer<T, P> {
    pub fn new() -> Self {
        Self { blocks: RwLock::new(AHashMapWrapper::new()) }
    }
}

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct ProximaGenericBlock<T: AD, P: O3DPose<T>> {
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    pose_a_j: P,
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    pose_b_j: P,
    #[serde_as(as = "SerdeO3DVec<T, <P::RotationType as O3DRotation<T>>::Native3DVecType>")]
    closest_point_a_j: <P::RotationType as O3DRotation<T>>::Native3DVecType,
    #[serde_as(as = "SerdeO3DVec<T, <P::RotationType as O3DRotation<T>>::Native3DVecType>")]
    closest_point_b_j: <P::RotationType as O3DRotation<T>>::Native3DVecType,
    #[serde_as(as = "SerdeAD<T>")]
    raw_distance_j: T,
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    displacement_between_a_and_b_j: P,
}