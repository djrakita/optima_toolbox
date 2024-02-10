
/*
use std::borrow::Cow;
use std::marker::PhantomData;
use ad_trait::AD;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DifferentiableFunctionClass, DifferentiableFunctionTrait};
use parry_ad::shape::Ball;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategory};
use optima_3d_spatial::optima_3d_vec::O3DVec;
use optima_interpolation::{get_interpolation_range_num_steps, InterpolatorTrait, InterpolatorTraitLite};
use optima_interpolation::splines::SplineConstructorTrait;
use optima_linalg::OVec;
use optima_optimization::loss_functions::{GrooveLossGaussianDirection, OptimizationLossFunctionTrait, OptimizationLossGroove};
use optima_proximity::pair_group_queries::{OPairGroupQryTrait, OwnedPairGroupQry, OParryPairSelector, OProximityLossFunction, ToParryProximityOutputCategory};
use optima_proximity::shape_scene::{OParryGenericShapeScene, ShapeSceneTrait};
use optima_proximity::shapes::{OParryShape, ShapeCategoryOParryShape};
use crate::robotics_optimization::robotics_optimization_functions::{min_acceleration_over_path_objective, min_jerk_over_path_objective, min_velocity_over_path_objective};

pub struct DifferentiableFunctionClassPathOpt<C: O3DPoseCategory, S: SplineConstructorTrait, Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=OParryPairSelector, OutputCategory=ToParryProximityOutputCategory>>(PhantomData<(C, S, Q)>);
impl<C: O3DPoseCategory, S: SplineConstructorTrait, Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=OParryPairSelector, OutputCategory=ToParryProximityOutputCategory>> DifferentiableFunctionClass for DifferentiableFunctionClassPathOpt<C, S, Q> {
    type FunctionType<'a, T: AD> = DifferentiableFunctionPathOpt<'a, T, C, S, Q>;
}

#[allow(dead_code)]
pub struct DifferentiableFunctionPathOpt<T: AD, C: O3DPoseCategory, S: SplineConstructorTrait, Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=OParryPairSelector, OutputCategory=ToParryProximityOutputCategory>>
{
    spline_constructor: S,
    environment: Cow<'a, OParryGenericShapeScene<T, C::P<T>>>,
    proximity_qry: OwnedPairGroupQry<T, Q>,
    spheres: Vec<OParryShape<T, C::P<T>>>,
    start_point: Vec<T>,
    end_point: Vec<T>,
    num_arclength_markers: usize,
    num_points_along_spline_to_sample: usize,
    sample_ts: Vec<T>,
    distance_cutoff: T,
    match_start_and_end_point_weight: T,
    collision_avoidance_weight: T,
    min_vel_weight: T,
    min_accel_weight: T,
    min_jerk_weight: T
}
impl<'a, T: AD, C: O3DPoseCategory, S: SplineConstructorTrait, Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=OParryPairSelector, OutputCategory=ToParryProximityOutputCategory>> DifferentiableFunctionPathOpt<'a, T, C, S, Q> {
    pub fn new(spline_constructor: S, environment: Cow<'a, OParryGenericShapeScene<T, C::P<T>>>, proximity_qry: OwnedPairGroupQry<'a, T, Q>, start_point: Vec<T>, end_point: Vec<T>, num_arclength_markers: usize, num_points_along_spline_to_sample: usize, match_start_and_end_point_weight: T, collision_avoidance_weight: T, min_vel_weight: T, min_accel_weight: T, min_jerk_weight: T) -> Self {
        let mut spheres = vec![];
        let dis = start_point.dis(&end_point);
        let num_spheres = num_points_along_spline_to_sample;
        let sphere_diameter = dis / T::constant(num_spheres as f64);
        let sphere_radius = (sphere_diameter / T::constant(2.0)) * T::constant(0.75);
        for _ in 0..num_spheres {
            spheres.push(OParryShape::new(Ball::new(sphere_radius), C::P::identity(), false, false));
        }
        let sample_ts = get_interpolation_range_num_steps(T::zero(), T::one(), num_spheres);
        let distance_cutoff = sphere_radius * T::constant(2.5);

        // let proximity_qry = OwnedParryDistanceAsProximityGroupQry::new(ParryDistanceGroupArgs::new(parry_shape_rep, ParryDisMode::ContactDis, false, false, T::constant(-10000.0), false));

        Self { spline_constructor, environment, proximity_qry, spheres, start_point, end_point, num_arclength_markers, num_points_along_spline_to_sample, sample_ts, distance_cutoff, match_start_and_end_point_weight, collision_avoidance_weight, min_vel_weight, min_accel_weight, min_jerk_weight }
    }
    pub fn to_other_ad_type<T1: AD>(&self) -> DifferentiableFunctionPathOpt<'a, T1, C, S, Q> {
        DifferentiableFunctionPathOpt {
            spline_constructor: self.spline_constructor.clone(),
            environment: Cow::Owned(self.environment.to_other_generic_types::<T1, C>()),
            proximity_qry: self.proximity_qry.to_other_ad_type::<T1>(),
            spheres: self.spheres.iter().map(|x| x.to_other_generic_category::<T1, C>() ).collect(),
            start_point: self.start_point.ovec_to_other_ad_type::<T1>(),
            end_point: self.end_point.ovec_to_other_ad_type::<T1>(),
            num_arclength_markers: self.num_arclength_markers,
            num_points_along_spline_to_sample: self.num_points_along_spline_to_sample,
            sample_ts: self.sample_ts.ovec_to_other_ad_type::<T1>(),
            distance_cutoff: self.distance_cutoff.to_other_ad_type::<T1>(),
            match_start_and_end_point_weight: self.match_start_and_end_point_weight.to_other_ad_type::<T1>(),
            collision_avoidance_weight: self.collision_avoidance_weight.to_other_ad_type::<T1>(),
            min_vel_weight: self.min_vel_weight.to_other_ad_type::<T1>(),
            min_accel_weight: self.min_accel_weight.to_other_ad_type::<T1>(),
            min_jerk_weight: self.min_jerk_weight.to_other_ad_type::<T1>(),
        }
    }
}

impl<'a, T: AD, C: O3DPoseCategory, S: SplineConstructorTrait, Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=OParryPairSelector, OutputCategory=ToParryProximityOutputCategory>> DifferentiableFunctionTrait<'a, T> for DifferentiableFunctionPathOpt<'a, T, C, S, Q> {
    fn call(&self, inputs: &[T], freeze: bool) -> Vec<T> {
        let mut out = T::zero();

        let control_points = inputs.to_vec().ovec_split_into_sub_vecs_owned(3);
        let spline = self.spline_constructor.construct(control_points).to_arclength_parameterized_interpolator(self.num_arclength_markers);

        let mut spline_points = vec![];
        self.sample_ts.iter().for_each(|t| {
            let spline_point = spline.interpolate(*t);
            spline_points.push(spline_point);
        });

        if self.match_start_and_end_point_weight > T::zero() {
            let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T::zero(), T::constant(2.0), T::constant(0.2), T::constant(1.0), T::constant(2.0));
            let start_dis = spline_points.first().as_ref().unwrap().ovec_sub(&self.start_point).norm();
            let end_dis = spline_points.last().as_ref().unwrap().ovec_sub(&self.start_point).norm();

            out += self.match_start_and_end_point_weight*loss.loss(start_dis);
            out += self.match_start_and_end_point_weight*loss.loss(end_dis);
        }

        if self.collision_avoidance_weight > T::zero() {
            let mut poses = vec![];
            spline_points.iter().for_each(|x| poses.push( C::P::from_constructors(x, &vec![T::zero(); 3]) ) );

            let s1 = &self.spheres;
            let s2 = self.environment.as_ref().get_shapes();

            let p1 = &poses;
            let p2 = self.environment.as_ref().get_shape_poses(&());

            let res = self.proximity_qry.query(&s1, &s2, &p1, p2.as_ref(), &OParryPairSelector::AllPairs, &(), &(), freeze);
            let proximity_value = res.get_proximity_objective_value(self.distance_cutoff, T::constant(15.0), OProximityLossFunction::Hinge);

            let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T::zero(), T::constant(6.0), T::constant(0.4), T::constant(2.0), T::constant(4.0));
            out += self.collision_avoidance_weight*loss.loss(proximity_value);
        }

        let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T::zero(), T::constant(2.0), T::constant(0.2), T::constant(2.0), T::constant(2.0));
        if self.min_vel_weight > T::zero() {
            out += self.min_vel_weight * loss.loss(min_velocity_over_path_objective(&spline_points, T::constant(10.0)));
        }
        if self.min_accel_weight > T::zero() {
            out += self.min_accel_weight * loss.loss(min_acceleration_over_path_objective(&spline_points, T::constant(10.0)));
        }
        if self.min_jerk_weight > T::zero() {
            out += self.min_jerk_weight * loss.loss(min_jerk_over_path_objective(&spline_points, T::constant(10.0)));
        }

        vec![out]
    }

    fn num_inputs(&self) -> usize {
        self.spheres.len() * 3
    }

    fn num_outputs(&self) -> usize {
        1
    }
}

pub type DifferentiableBlockPathOpt<'a, C, S, Q, E> = DifferentiableBlock<'a, DifferentiableFunctionClassPathOpt<C, S, Q>, E>;
*/