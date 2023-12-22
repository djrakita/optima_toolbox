use std::sync::RwLock;
use std::time::{Duration, Instant};
use ad_trait::AD;
use parry_ad::na::{Isometry3, Vector3};
use serde::{Deserialize, Serialize};
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_3d_spatial::optima_3d_rotation::O3DRotation;
use optima_universal_hashmap::AHashMapWrapper;
use crate::pair_group_queries::{get_parry_ids_from_shape_pair, OPairGroupQryTrait, OwnedPairGroupQry, PairAverageDistanceTrait, PairGroupQryArgsCategory, PairGroupQryOutputCategory, PairSkipsTrait, parry_generic_pair_group_query, ParryContactGroupArgs, ParryContactGroupQry, ParryPairGroupOutputWrapper, ParryPairIdxs, ParryPairSelector, ProximityLossFunction, ToParryProximityOutputCategory, ToParryProximityOutputTrait};
use crate::pair_queries::{OPairQryTrait, ParryOutputAuxData, ParryProximaDistanceBoundsArgs, ParryProximaDistanceBoundsOutputOption, ParryProximaDistanceBoundsQry, ParryQryShapeType, ParryShapeRep};
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

    fn query<'a, T: AD, P: O3DPose<T>, S: PairSkipsTrait, A: PairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, freeze: bool, args: &<Self::ArgsCategory as PairGroupQryArgsCategory>::Args<'a, T>) -> <Self::OutputCategory as PairGroupQryOutputCategory>::Output<T, P> {
        let start = Instant::now();
        if !freeze { args.proxima_container.transfer_staging_to_current_for_all_blocks(); }

        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryProximaDistanceBoundsOutputOption<T, P> {
            let ids = get_parry_ids_from_shape_pair(shape_a, shape_b, parry_qry_shape_type, parry_shape_rep);
            let average_distance = match args.use_average_distance {
                true => { Some(pair_average_distances.average_distance(ids.0, ids.1)) }
                false => { None }
            };
            let binding = args.proxima_container.blocks.read();
            let blocks = binding.as_ref().unwrap();
            let block = blocks.hashmap.get(&ids);
            return match block {
                None => {
                    drop(binding);
                    let average_distance = match args.use_average_distance {
                        true => { Some(pair_average_distances.average_distance(ids.0, ids.1)) }
                        false => { None }
                    };
                    let displacement_between_a_and_b_j = pose_a.displacement(pose_b);
                    let c = shape_a.contact(shape_b, pose_a, pose_b, &(T::constant(f64::MAX), parry_qry_shape_type.clone(), parry_shape_rep.clone(), average_distance));
                    let contact = c.contact.expect("error");
                    let mut binding = args.proxima_container.blocks.write();
                    let blocks = binding.as_mut().unwrap();
                    blocks.hashmap.insert(ids, ProximaGenericBlock {
                        dirty_flag: false,
                        pose_a_j_staging: pose_a.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().o3dpose_to_constant_ads(),
                        pose_b_j_staging: pose_b.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().o3dpose_to_constant_ads(),
                        closest_point_a_j_staging: contact.point1.coords.o3dvec_downcast_or_convert::<Vector3<T>>().o3dvec_to_constant_ads(),
                        closest_point_b_j_staging: contact.point2.coords.o3dvec_downcast_or_convert::<Vector3<T>>().o3dvec_to_constant_ads(),
                        raw_distance_j_staging: contact.dist.to_constant_ad(),
                        displacement_between_a_and_b_j_staging: displacement_between_a_and_b_j.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().o3dpose_to_constant_ads(),
                        pose_a_j: pose_a.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().o3dpose_to_constant_ads(),
                        pose_b_j: pose_b.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().o3dpose_to_constant_ads(),
                        closest_point_a_j: contact.point1.coords.o3dvec_downcast_or_convert::<Vector3<T>>().o3dvec_to_constant_ads(),
                        closest_point_b_j: contact.point2.coords.o3dvec_downcast_or_convert::<Vector3<T>>().o3dvec_to_constant_ads(),
                        raw_distance_j: contact.dist.to_constant_ad(),
                        displacement_between_a_and_b_j: displacement_between_a_and_b_j.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().o3dpose_to_constant_ads(),
                    });

                    ParryProximaDistanceBoundsQry::query(shape_a, shape_b, pose_a, pose_b, &ParryProximaDistanceBoundsArgs {
                        parry_qry_shape_type: parry_qry_shape_type.clone(),
                        parry_shape_rep: parry_shape_rep.clone(),
                        pose_a_j: pose_a.o3dpose_downcast_or_convert::<P>().as_ref(),
                        pose_b_j: pose_b.o3dpose_downcast_or_convert::<P>().as_ref(),
                        closest_point_a_j: contact.point1.coords.o3dvec_downcast_or_convert::<<P::RotationType as O3DRotation<T>>::Native3DVecType>().as_ref(),
                        closest_point_b_j: contact.point2.coords.o3dvec_downcast_or_convert::<<P::RotationType as O3DRotation<T>>::Native3DVecType>().as_ref(),
                        raw_distance_j: contact.dist,
                        displacement_between_a_and_b_j: &displacement_between_a_and_b_j,
                        cutoff_distance: args.cutoff_distance,
                        average_distance,
                    })
                }
                Some(block) => {
                    ParryProximaDistanceBoundsQry::query(shape_a, shape_b, pose_a, pose_b, &ParryProximaDistanceBoundsArgs {
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
                    })
                }
            };
        };

        let termination = |_o: &ParryProximaDistanceBoundsOutputOption<T, P>| { false };

        let (outputs, _) = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, &args.parry_shape_rep, pair_skips, args.for_filter, f, termination);

        let mut v = vec![];
        let mut proximity_lower_bound_sum = T::zero();
        let mut proximity_upper_bound_sum = T::zero();
        outputs.iter().for_each(|x| {
            match &x.data().0 {
                None => {}
                Some(data) => {
                    let lower_bound_through_loss_and_powf = args.loss_function.loss(data.distance_upper_bound_wrt_average, args.cutoff_distance).powf(args.p_norm);
                    let upper_bound_through_loss_and_powf = args.loss_function.loss(data.distance_lower_bound_wrt_average, args.cutoff_distance).powf(args.p_norm);
                    let diff = upper_bound_through_loss_and_powf - lower_bound_through_loss_and_powf;
                    proximity_lower_bound_sum += lower_bound_through_loss_and_powf;
                    proximity_upper_bound_sum += upper_bound_through_loss_and_powf;
                    // &(T, T, T, &P, (u64, u64), &ParryPairIdxs)
                    // let idx = v.binary_search_by(|x: &(T, T, T, &P, (u64, u64), &ParryPairIdxs)| diff.partial_cmp(&x.0).expect(&format!("{:?}, {:?}", diff, x.0)));
                    // let idx = match idx {
                    //     Ok(idx) => { idx }
                    //     Err(idx) => { idx }
                    // };
                    // v.insert(idx, (diff, lower_bound_through_loss_and_powf, upper_bound_through_loss_and_powf, &data.displacement_between_a_and_b_k, x.pair_ids(), x.pair_idxs()));
                    v.push((diff, lower_bound_through_loss_and_powf, upper_bound_through_loss_and_powf, &data.displacement_between_a_and_b_k, x.pair_ids(), x.pair_idxs()));
                }
            }
        });

        let mut indices = (0..v.len()).collect::<Vec<_>>();
        indices.sort_by(|x, y| v[*x].0.partial_cmp(&v[*y].0).unwrap());
        // v.sort_by(|x, y| { y.0.partial_cmp(&x.0).expect(&format!("y.1 {:?}, y.0 {:?}, x.1 {:?}, x.0 {:?}", y.1, y.0, x.1, x.0)) });

        let mut proximity_lower_bound_output;
        let mut proximity_upper_bound_output;
        let mut max_possible_error;

        let mut num_queries = 0;

        let mut idx = indices.len();
        'l: loop {
            proximity_lower_bound_output = proximity_lower_bound_sum.powf(args.p_norm.recip());
            proximity_upper_bound_output = proximity_upper_bound_sum.powf(args.p_norm.recip());
            // assert!(proximity_upper_bound_output >= proximity_lower_bound_output);
            max_possible_error = (proximity_upper_bound_output - proximity_lower_bound_output).abs();

            if num_queries >= v.len() { break 'l; }
            if idx == 0 { break 'l; }
            idx -= 1;

            let mut terminate = false;
            match &args.termination {
                ProximaTermination::MaxTime(t) => {
                    if start.elapsed() > *t { terminate = true; }
                }
                ProximaTermination::MaxError(e) => {
                    if max_possible_error.to_constant() < *e { terminate = true; }
                }
            }

            if terminate { break 'l; }

            let curr_entry = &v[idx];
            let lower_bound_through_loss_and_powf = curr_entry.1;
            let upper_bound_through_loss_and_powf = curr_entry.2;
            let displacement_between_a_and_b_k = curr_entry.3;
            let ids = curr_entry.4;
            let idxs = curr_entry.5;
            let poses = match idxs {
                ParryPairIdxs::Shapes(i, j) => { (&poses_a[*i], &poses_b[*j]) }
                ParryPairIdxs::ShapeSubcomponents((i, _), (j, _)) => { (&poses_a[*i], &poses_b[*j]) }
            };

            let selector = ParryPairSelector::PairsByIdxs(vec![idxs.clone()]);

            let res = ParryContactGroupQry::query(shape_group_a, shape_group_b, poses_a, poses_b, &selector, pair_skips, pair_average_distances, false, &ParryContactGroupArgs::new(args.parry_shape_rep.clone(), T::constant(f64::MAX), args.use_average_distance, args.for_filter, T::constant(f64::MIN)));
            let output = &res.outputs()[0];
            let data = output.data();
            let distance_wrt_average = data.distance_wrt_average.expect("this should never be none");
            let contact = data.contact().expect("this should never be none");
            let raw_distance = &contact.dist;
            num_queries += 1;

            let ground_truth_through_loss_and_powf = args.loss_function.loss(distance_wrt_average, args.cutoff_distance).powf(args.p_norm);
            proximity_lower_bound_sum -= lower_bound_through_loss_and_powf;
            proximity_upper_bound_sum -= upper_bound_through_loss_and_powf;
            proximity_lower_bound_sum += ground_truth_through_loss_and_powf;
            proximity_upper_bound_sum += ground_truth_through_loss_and_powf;

            let mut binding = args.proxima_container.blocks.write().unwrap();
            let block = binding.hashmap.get_mut(&ids).expect("error");
            block.dirty_flag = true;
            block.pose_a_j_staging = poses.0.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().o3dpose_to_constant_ads();
            block.pose_b_j_staging = poses.1.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().o3dpose_to_constant_ads();
            block.closest_point_a_j_staging = contact.point1.o3dvec_downcast_or_convert::<Vector3<T>>().as_ref().o3dvec_to_constant_ads();
            block.closest_point_b_j_staging = contact.point2.o3dvec_downcast_or_convert::<Vector3<T>>().as_ref().o3dvec_to_constant_ads();
            block.raw_distance_j_staging = raw_distance.to_constant_ad();
            block.displacement_between_a_and_b_j_staging = displacement_between_a_and_b_k.o3dpose_downcast_or_convert::<Isometry3<T>>().as_ref().o3dpose_to_constant_ads();

            /*
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
        */
        }

        Box::new(ParryProximaGroupOutput {
            output_proximity_value: T::constant(0.01)*proximity_upper_bound_output + T::constant(0.99)*proximity_lower_bound_output,
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
impl PairGroupQryOutputCategory for PairGroupQryOutputCategoryParryProxima {
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

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct ParryProximaAsProximityQry;
impl OPairGroupQryTrait for ParryProximaAsProximityQry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = ParryPairSelector;
    type ArgsCategory = PairGroupQryArgsCategoryParryProxima;
    type OutputCategory = ToParryProximityOutputCategory;

    fn query<'a, T: AD, P: O3DPose<T>, S: PairSkipsTrait, A: PairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, freeze: bool, args: &<Self::ArgsCategory as PairGroupQryArgsCategory>::Args<'a, T>) -> <Self::OutputCategory as PairGroupQryOutputCategory>::Output<T, P> {
        ParryProximaQry::query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, pair_skips, pair_average_distances, freeze, args)
    }
}

pub type OwnedParryProximaAsProximityQry<'a, T> = OwnedPairGroupQry<'a, T, ParryProximaAsProximityQry>;

////////////////////////////////////////////////////////////////////////////////////////////////////

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct PairGroupQryArgsParryProxima<T: AD> {
    #[serde(deserialize_with = "ProximaGenericContainer::<T, Isometry3::<T>>::deserialize")]
    proxima_container: ProximaGenericContainer<T, Isometry3<T>>,
    parry_shape_rep: ParryShapeRep,
    use_average_distance: bool,
    for_filter: bool,
    termination: ProximaTermination,
    loss_function: ProximityLossFunction,
    #[serde_as(as = "SerdeAD<T>")]
    p_norm: T,
    #[serde_as(as = "SerdeAD<T>")]
    cutoff_distance: T
}
impl<T: AD> PairGroupQryArgsParryProxima<T> {
    pub fn new(parry_shape_rep: ParryShapeRep, use_average_distance: bool, for_filter: bool, termination: ProximaTermination, loss_function: ProximityLossFunction, p_norm: T, cutoff_distance: T) -> Self {
        let proxima_container = ProximaGenericContainer::new();

        Self { proxima_container, parry_shape_rep, use_average_distance, for_filter, termination, loss_function, p_norm, cutoff_distance }
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
    pub fn transfer_staging_to_current_for_all_blocks(&self) {
        let mut binding = self.blocks.write();
        let a = binding.as_mut().unwrap();
        a.hashmap.values_mut().for_each(|x| { x.transfer_staging_to_current(); });
    }
    pub fn get_outputs<S: PairSkipsTrait, A: PairAverageDistanceTrait<T>>(&self,
                                                                          shape_group_a: &Vec<OParryShape<T, P>>,
                                                                          shape_group_b: &Vec<OParryShape<T, P>>,
                                                                          poses_a: &Vec<P>,
                                                                          poses_b: &Vec<P>,
                                                                          pair_selector: &ParryPairSelector,
                                                                          parry_shape_rep: &ParryShapeRep,
                                                                          pair_skips: &S,
                                                                          pair_average_distances: &A,
                                                                          for_filter: bool,
                                                                          use_average_distances: bool,
                                                                          cutoff_distance: T) -> Vec<ParryPairGroupOutputWrapper<ParryProximaDistanceBoundsOutputOption<T, P>>> {
        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryProximaDistanceBoundsOutputOption<T, P> {
            let ids = get_parry_ids_from_shape_pair(shape_a, shape_b, parry_qry_shape_type, parry_shape_rep);
            let average_distance = match use_average_distances {
                true => { Some(pair_average_distances.average_distance(ids.0, ids.1)) }
                false => { None }
            };
            let binding = self.blocks.read();
            let blocks = binding.as_ref().unwrap();
            let block = blocks.hashmap.get(&ids);
            return match block {
                None => {
                    drop(binding);
                    let average_distance = match use_average_distances {
                        true => { Some(pair_average_distances.average_distance(ids.0, ids.1)) }
                        false => { None }
                    };
                    let displacement_between_a_and_b_j = pose_a.displacement(pose_b);
                    let c = shape_a.contact(shape_b, pose_a, pose_b, &(T::constant(f64::MAX), parry_qry_shape_type.clone(), parry_shape_rep.clone(), average_distance));
                    let contact = c.contact.expect("error");
                    let mut binding = self.blocks.write();
                    let blocks = binding.as_mut().unwrap();
                    blocks.hashmap.insert(ids, ProximaGenericBlock {
                        dirty_flag: false,
                        pose_a_j_staging: pose_a.o3dpose_downcast_or_convert::<P>().as_ref().o3dpose_to_constant_ads(),
                        pose_b_j_staging: pose_b.o3dpose_downcast_or_convert::<P>().as_ref().o3dpose_to_constant_ads(),
                        closest_point_a_j_staging: contact.point1.coords.o3dvec_downcast_or_convert::<<P::RotationType as O3DRotation<T>>::Native3DVecType>().o3dvec_to_constant_ads(),
                        closest_point_b_j_staging: contact.point2.coords.o3dvec_downcast_or_convert::<<P::RotationType as O3DRotation<T>>::Native3DVecType>().o3dvec_to_constant_ads(),
                        raw_distance_j_staging: contact.dist.to_constant_ad(),
                        displacement_between_a_and_b_j_staging: displacement_between_a_and_b_j.o3dpose_downcast_or_convert::<P>().as_ref().o3dpose_to_constant_ads(),
                        pose_a_j: pose_a.o3dpose_downcast_or_convert::<P>().as_ref().o3dpose_to_constant_ads(),
                        pose_b_j: pose_b.o3dpose_downcast_or_convert::<P>().as_ref().o3dpose_to_constant_ads(),
                        closest_point_a_j: contact.point1.coords.o3dvec_downcast_or_convert::<<P::RotationType as O3DRotation<T>>::Native3DVecType>().o3dvec_to_constant_ads(),
                        closest_point_b_j: contact.point2.coords.o3dvec_downcast_or_convert::<<P::RotationType as O3DRotation<T>>::Native3DVecType>().o3dvec_to_constant_ads(),
                        raw_distance_j: contact.dist.to_constant_ad(),
                        displacement_between_a_and_b_j: displacement_between_a_and_b_j.o3dpose_downcast_or_convert::<P>().as_ref().o3dpose_to_constant_ads(),
                    });

                    ParryProximaDistanceBoundsQry::query(shape_a, shape_b, pose_a, pose_b, &ParryProximaDistanceBoundsArgs {
                        parry_qry_shape_type: parry_qry_shape_type.clone(),
                        parry_shape_rep: parry_shape_rep.clone(),
                        pose_a_j: pose_a.o3dpose_downcast_or_convert::<P>().as_ref(),
                        pose_b_j: pose_b.o3dpose_downcast_or_convert::<P>().as_ref(),
                        closest_point_a_j: contact.point1.coords.o3dvec_downcast_or_convert::<<P::RotationType as O3DRotation<T>>::Native3DVecType>().as_ref(),
                        closest_point_b_j: contact.point2.coords.o3dvec_downcast_or_convert::<<P::RotationType as O3DRotation<T>>::Native3DVecType>().as_ref(),
                        raw_distance_j: contact.dist,
                        displacement_between_a_and_b_j: &displacement_between_a_and_b_j,
                        cutoff_distance,
                        average_distance,
                    })
                }
                Some(block) => {
                    ParryProximaDistanceBoundsQry::query(shape_a, shape_b, pose_a, pose_b, &ParryProximaDistanceBoundsArgs {
                        parry_qry_shape_type: parry_qry_shape_type.clone(),
                        parry_shape_rep: parry_shape_rep.clone(),
                        pose_a_j: &block.pose_a_j,
                        pose_b_j: &block.pose_b_j,
                        closest_point_a_j: &block.closest_point_a_j,
                        closest_point_b_j: &block.closest_point_b_j,
                        raw_distance_j: block.raw_distance_j,
                        displacement_between_a_and_b_j: &block.displacement_between_a_and_b_j,
                        cutoff_distance,
                        average_distance,
                    })
                }
            };
        };

        let termination = |_o: &ParryProximaDistanceBoundsOutputOption<T, P>| { false };

        let outputs = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, parry_shape_rep, pair_skips, for_filter, f, termination);

        outputs.0
    }
}

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct ProximaGenericBlock<T: AD, P: O3DPose<T>> {
    dirty_flag: bool,
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    pose_a_j_staging: P,
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    pose_b_j_staging: P,
    #[serde_as(as = "SerdeO3DVec<T, <P::RotationType as O3DRotation<T>>::Native3DVecType>")]
    closest_point_a_j_staging: <P::RotationType as O3DRotation<T>>::Native3DVecType,
    #[serde_as(as = "SerdeO3DVec<T, <P::RotationType as O3DRotation<T>>::Native3DVecType>")]
    closest_point_b_j_staging: <P::RotationType as O3DRotation<T>>::Native3DVecType,
    #[serde_as(as = "SerdeAD<T>")]
    raw_distance_j_staging: T,
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    displacement_between_a_and_b_j_staging: P,
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
    displacement_between_a_and_b_j: P
}
impl<T: AD, P: O3DPose<T>> ProximaGenericBlock<T, P> {
    #[inline(always)]
    pub fn transfer_staging_to_current(&mut self) {
        if self.dirty_flag {
            self.pose_a_j = self.pose_a_j_staging.clone();
            self.pose_b_j = self.pose_b_j_staging.clone();
            self.closest_point_a_j = self.closest_point_a_j_staging.clone();
            self.closest_point_b_j = self.closest_point_b_j_staging.clone();
            self.raw_distance_j = self.raw_distance_j_staging.clone();
            self.displacement_between_a_and_b_j = self.displacement_between_a_and_b_j_staging.clone();
            self.dirty_flag = false;
        }
    }
}