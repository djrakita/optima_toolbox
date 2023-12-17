/*
use std::sync::RwLock;
use std::time::{Duration, Instant};
use ad_trait::AD;
use serde::{Deserialize, Serialize};
use serde_with::serde_as;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use crate::pair_group_queries::{get_parry_ids_from_shape_pair, OPairGroupQryTrait, PairAverageDistanceTrait, PairGroupQryArgsCategory, PairGroupQryOutputCategory, PairSkipsTrait, parry_generic_pair_group_query, ParryPairGroupOutputWrapper, ParryPairSelector, ProximityLossFunction, ToParryProximityOutputTrait};
use crate::shapes::{OParryShape, ShapeCategoryOParryShape, ShapeCategoryTrait};
use ad_trait::SerdeAD;
use parry3d_f64::na::{Isometry3};
use optima_3d_spatial::optima_3d_pose::SerdeO3DPose;
use optima_universal_hashmap::AHashMapWrapper;
use crate::pair_queries::{OPairQryTrait, ParryOutputAuxData, ParryProxima2DistanceBoundsArgs, ParryProxima2DistanceBoundsQry, ParryProximaDistanceBoundsOutput, ParryQryShapeType, ParryShapeRep};
use crate::shape_queries::OShpQryContactTrait;

pub struct ParryProxima2Qry;
impl OPairGroupQryTrait for ParryProxima2Qry {
    type ShapeCategory = ShapeCategoryOParryShape;
    type SelectorType = ParryPairSelector;
    type ArgsCategory = PairGroupQryArgsCategoryParryProxima;
    type OutputCategory = PairGroupQryOutputCategoryParryProxima2;

    fn query<'a, T: AD, P: O3DPose<T>, S: PairSkipsTrait, A: PairAverageDistanceTrait<T>>(shape_group_a: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, shape_group_b: &Vec<<Self::ShapeCategory as ShapeCategoryTrait>::ShapeType<T, P>>, poses_a: &Vec<P>, poses_b: &Vec<P>, pair_selector: &Self::SelectorType, pair_skips: &S, pair_average_distances: &A, freeze: bool, args: &<Self::ArgsCategory as PairGroupQryArgsCategory>::Args<'a, T>) -> <Self::OutputCategory as PairGroupQryOutputCategory>::Output<T, P> {
        let start = Instant::now();
        if !freeze { args.proxima_container.transfer_staging_to_current_for_all_blocks(); }



        todo!()
    }
}

pub struct ParryProxima2GroupOutput<T: AD> {
    output_proximity_value: T,
    estimated_maximum_possible_proximity_value_error: T,
    aux_data: ParryOutputAuxData
}
impl<T: AD> ParryProxima2GroupOutput<T> {
    #[inline(always)]
    pub fn output_proximity_value(&self) -> T {
        self.output_proximity_value
    }
    #[inline(always)]
    pub fn estimated_maximum_possible_proximity_value_error(&self) -> &T {
        &self.estimated_maximum_possible_proximity_value_error
    }
    #[inline(always)]
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl<T: AD> ToParryProximityOutputTrait<T> for ParryProxima2GroupOutput<T> {
    fn get_proximity_objective_value(&self, _cutoff: T, _p_norm: T, _loss_function: ProximityLossFunction) -> T {
        self.output_proximity_value
    }
}

pub struct PairGroupQryOutputCategoryParryProxima2;
impl PairGroupQryOutputCategory for PairGroupQryOutputCategoryParryProxima2 {
    type Output<T: AD, P: O3DPose<T>> = ParryProxima2GroupOutput<T>;
}

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct PairGroupQryArgsParryProxima2<T: AD> {
    #[serde(deserialize_with = "Proxima2GenericContainer::<T, Isometry3::<T>>::deserialize")]
    proxima_container: Proxima2GenericContainer<T, Isometry3<T>>,
    parry_shape_rep: ParryShapeRep,
    use_average_distance: bool,
    for_filter: bool,
    termination: ProximaTermination2,
    loss_function: ProximityLossFunction,
    #[serde_as(as = "SerdeAD<T>")]
    p_norm: T,
    #[serde_as(as = "SerdeAD<T>")]
    cutoff_distance: T
}
impl<T: AD> PairGroupQryArgsParryProxima2<T> {
    pub fn new(parry_shape_rep: ParryShapeRep, use_average_distance: bool, for_filter: bool, termination: ProximaTermination2, loss_function: ProximityLossFunction, p_norm: T, cutoff_distance: T) -> Self {
        Self { proxima_container: Proxima2GenericContainer::new(), parry_shape_rep, use_average_distance, for_filter, termination, loss_function, p_norm, cutoff_distance }
    }
}

pub struct PairGroupQryArgsCategoryParryProxima;
impl PairGroupQryArgsCategory for PairGroupQryArgsCategoryParryProxima {
    type Args<'a, T: AD> = PairGroupQryArgsParryProxima2<T>;
    type QueryType = ParryProxima2Qry;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ProximaTermination2 {
    MaxTime(Duration),
    MaxError(f64)
}

#[derive(Serialize, Deserialize)]
pub struct Proxima2GenericContainer<T: AD, P: O3DPose<T>> {
    #[serde(deserialize_with = "RwLock::<AHashMapWrapper::<(u64, u64), Proxima2GenericBlock::<T, P>>>::deserialize")]
    blocks: RwLock<AHashMapWrapper<(u64, u64), Proxima2GenericBlock<T, P>>>
}
impl<T: AD, P: O3DPose<T>> Proxima2GenericContainer<T, P> {
    pub fn new() -> Self {
        Self { blocks: RwLock::new(AHashMapWrapper::new()) }
    }
    pub fn transfer_staging_to_current_for_all_blocks(&self) {
        let mut binding = self.blocks.write();
        let a = binding.as_mut().unwrap();
        a.hashmap.values_mut().for_each(|x| {
            if x.dirty_flag {
                x.transfer_staging_to_current();
                x.dirty_flag = false;
            }
        });
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
                                                                          cutoff_distance: T) -> Vec<ParryPairGroupOutputWrapper<ParryProximaDistanceBoundsOutput<T, P>>> {
        let f = |shape_a: &OParryShape<T, P>, shape_b: &OParryShape<T, P>, pose_a: &P, pose_b: &P, parry_qry_shape_type: &ParryQryShapeType, parry_shape_rep: &ParryShapeRep| -> ParryProximaDistanceBoundsOutput<T, P> {
            let ids = get_parry_ids_from_shape_pair(shape_a, shape_b, parry_qry_shape_type, parry_shape_rep);
            let average_distance = match use_average_distances {
                true => { Some(pair_average_distances.average_distance(ids.0, ids.1)) }
                false => { None }
            };
            let binding = self.blocks.read();
            let blocks = binding.as_ref().unwrap();
            let block = blocks.hashmap.get(&ids);
            match block {
                None => {
                    drop(binding);
                    let displacement_between_a_and_b_j = pose_a.displacement(pose_b);
                    let c = shape_a.contact(shape_b, pose_a, pose_b, &(T::constant(f64::MAX), parry_qry_shape_type.clone(), parry_shape_rep.clone(), average_distance));
                    let contact = c.contact.expect("error");

                    let pose_a_k_fake = pose_a.mul(&P::from_constructors(&[T::constant(0.01),T::constant(0.01),T::constant(0.01)], &[T::constant(0.01),T::constant(0.01),T::constant(0.01)]));
                    let pose_b_k_fake = pose_b.mul(&P::from_constructors(&[T::constant(-0.02),T::constant(-0.02),T::constant(-0.02)], &[T::constant(-0.02),T::constant(-0.02),T::constant(-0.02)]));
                    let c_fake = shape_a.contact(shape_b, &pose_a_k_fake, &pose_b_k_fake, &(T::constant(f64::MAX), parry_qry_shape_type.clone(), parry_shape_rep.clone(), average_distance));
                    let contact_fake = c_fake.contact.expect("error");

                    let displacement_between_a_and_b_k_fake = pose_a_k_fake.displacement(&pose_b_k_fake);
                    let disp_of_disps = displacement_between_a_and_b_j.displacement(&displacement_between_a_and_b_k_fake);
                    let mag = disp_of_disps.magnitude();
                    let dis = (contact.dist - contact_fake.dist).abs();
                    let dis_per_unit = dis / mag;

                    let mut binding = self.blocks.write();
                    let blocks = binding.as_mut().unwrap();
                    blocks.hashmap.insert(ids, Proxima2GenericBlock {
                        dirty_flag: false,
                        raw_distance_j_staging: contact.dist,
                        raw_distance_j: contact.dist,
                        max_delta_distance_per_unit_staging: dis_per_unit,
                        max_delta_distance_per_unit: dis_per_unit,
                        displacement_between_a_and_b_j_staging: displacement_between_a_and_b_j.o3dpose_downcast_or_convert::<P>().as_ref().o3dpose_to_constant_ads(),
                        displacement_between_a_and_b_j: displacement_between_a_and_b_j.o3dpose_downcast_or_convert::<P>().as_ref().o3dpose_to_constant_ads(),
                    });
                }
                Some(block) => {
                    ParryProxima2DistanceBoundsQry::query(shape_a, shape_b, pose_a, pose_b, &ParryProxima2DistanceBoundsArgs {
                        parry_qry_shape_type: parry_qry_shape_type.clone(),
                        parry_shape_rep: parry_shape_rep.clone(),
                        raw_distance_j: block.raw_distance_j,
                        displacement_between_a_and_b_j: &block.displacement_between_a_and_b_j,
                        max_delta_distance_per_unit: block.max_delta_distance_per_unit,
                        cutoff_distance,
                        average_distance,
                    });
                }
            }

            todo!()
        };

        let termination = |_o: &ParryProximaDistanceBoundsOutput<T, P>| { false };

        let outputs = parry_generic_pair_group_query(shape_group_a, shape_group_b, poses_a, poses_b, pair_selector, parry_shape_rep, pair_skips, for_filter, f, termination);

        outputs.0
    }
}

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct Proxima2GenericBlock<T: AD, P: O3DPose<T>> {
    dirty_flag: bool,
    #[serde_as(as = "SerdeAD<T>")]
    raw_distance_j_staging: T,
    #[serde_as(as = "SerdeAD<T>")]
    raw_distance_j: T,
    #[serde_as(as = "SerdeAD<T>")]
    max_delta_distance_per_unit_staging: T,
    #[serde_as(as = "SerdeAD<T>")]
    max_delta_distance_per_unit: T,
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    displacement_between_a_and_b_j_staging: P,
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    displacement_between_a_and_b_j: P,
}
impl<T: AD, P: O3DPose<T>> Proxima2GenericBlock<T, P> {
    #[inline(always)]
    pub fn transfer_staging_to_current(&mut self) {
        self.raw_distance_j = self.raw_distance_j_staging;
        self.max_delta_distance_per_unit = self.max_delta_distance_per_unit_staging;
        self.displacement_between_a_and_b_j = self.displacement_between_a_and_b_j_staging.clone();
    }
}
*/
