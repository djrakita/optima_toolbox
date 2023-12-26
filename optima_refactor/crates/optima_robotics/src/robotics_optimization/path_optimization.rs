use std::borrow::Cow;
use ad_trait::AD;
use ad_trait::differentiable_function::DifferentiableFunctionTrait;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategory};
use optima_3d_spatial::optima_3d_vec::O3DVec;
use optima_interpolation::{InterpolatorTrait, InterpolatorTraitLite};
use optima_interpolation::splines::SplineConstructorTrait;
use optima_linalg::OVec;
use optima_optimization::loss_functions::{GrooveLossGaussianDirection, OptimizationLossFunctionTrait, OptimizationLossGroove};
use optima_proximity::pair_group_queries::{OPairGroupQryTrait, OwnedPairGroupQry, ParryPairSelector, ProximityLossFunction, ToParryProximityOutputCategory};
use optima_proximity::shape_scene::{OParryGenericShapeScene, ShapeSceneTrait};
use optima_proximity::shapes::{OParryShape, ShapeCategoryOParryShape};

pub struct DifferentiableFunctionPathOpt<'a, T: AD, C: O3DPoseCategory, S: SplineConstructorTrait, Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=ToParryProximityOutputCategory>>
{
    spline_constructor: S,
    environment: Cow<'a, OParryGenericShapeScene<T, C::P<T>>>,
    proximity_qry: OwnedPairGroupQry<'a, T, Q>,
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

impl<'a, T: AD, C: O3DPoseCategory, S: SplineConstructorTrait, Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=ToParryProximityOutputCategory>> DifferentiableFunctionTrait<'a, T> for DifferentiableFunctionPathOpt<'a, T, C, S, Q> {
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

            let res = self.proximity_qry.query(&s1, &s2, &p1, p2.as_ref(), &ParryPairSelector::AllPairs, &(), &(), freeze);
            let proximity_value = res.get_proximity_objective_value(self.distance_cutoff, T::constant(15.0), ProximityLossFunction::Hinge);

            let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T::zero(), T::constant(6.0), T::constant(0.4), T::constant(2.0), T::constant(4.0));
            out += self.collision_avoidance_weight*loss.loss(proximity_value);
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
