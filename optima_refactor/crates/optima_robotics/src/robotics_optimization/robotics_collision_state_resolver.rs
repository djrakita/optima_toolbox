use std::marker::PhantomData;
use std::sync::Arc;
use ad_trait::AD;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::{DifferentiableFunctionClass, DifferentiableFunctionTrait};
use optima_3d_spatial::optima_3d_pose::O3DPoseCategory;
use optima_3d_spatial::optima_3d_vec::O3DVec;
use optima_linalg::{OLinalgCategory, OVec};
use optima_optimization::loss_functions::{GrooveLossGaussianDirection, OptimizationLossFunctionTrait, OptimizationLossGroove};
use optima_proximity::pair_group_queries::{OPairGroupQryTrait, OwnedPairGroupQry, ParryPairSelector, ProximityLossFunction, ToParryProximityOutputCategory};
use optima_proximity::shape_scene::OParryGenericShapeScene;
use optima_proximity::shapes::ShapeCategoryOParryShape;
use crate::robot::ORobot;

pub struct DifferentiableFunctionClassCollisionStateResolver<C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static, Q1, Q2>(PhantomData<(C, L, Q1, Q2)>)
    where Q1: OPairGroupQryTrait<SelectorType=ParryPairSelector, ShapeCategory=ShapeCategoryOParryShape, OutputCategory=ToParryProximityOutputCategory>,
          Q2: OPairGroupQryTrait<SelectorType=ParryPairSelector, ShapeCategory=ShapeCategoryOParryShape, OutputCategory=ToParryProximityOutputCategory>;
impl<C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static, Q1, Q2> DifferentiableFunctionClass for DifferentiableFunctionClassCollisionStateResolver<C, L, Q1, Q2>
    where Q1: OPairGroupQryTrait<SelectorType=ParryPairSelector, ShapeCategory=ShapeCategoryOParryShape, OutputCategory=ToParryProximityOutputCategory>,
          Q2: OPairGroupQryTrait<SelectorType=ParryPairSelector, ShapeCategory=ShapeCategoryOParryShape, OutputCategory=ToParryProximityOutputCategory>
{
    type FunctionType<'a, T: AD> = DifferentiableFunctionCollisionStateResolver<'a, T, C, L, Q1, Q2>;
}

pub struct DifferentiableFunctionCollisionStateResolver<'a, T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static, Q1, Q2>
    where Q1: OPairGroupQryTrait<SelectorType=ParryPairSelector, ShapeCategory=ShapeCategoryOParryShape, OutputCategory=ToParryProximityOutputCategory>,
          Q2: OPairGroupQryTrait<SelectorType=ParryPairSelector, ShapeCategory=ShapeCategoryOParryShape, OutputCategory=ToParryProximityOutputCategory>
{
    robot: Arc<ORobot<T, C, L>>,
    environment: Arc<OParryGenericShapeScene<T, C::P<T>>>,
    start_state: Vec<T>,
    boundary_point_a: Vec<T>,
    boundary_point_b: Vec<T>,
    self_proximity_query: OwnedPairGroupQry<'a, T, Q1>,
    environment_proximity_query: OwnedPairGroupQry<'a, T, Q2>,
    self_selector: ParryPairSelector,
    environment_selector: ParryPairSelector,
    distance_cutoff: T,
    p_norm: T
}
impl<'a, T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static, Q1, Q2> DifferentiableFunctionCollisionStateResolver<'a, T, C, L, Q1, Q2> where Q1: OPairGroupQryTrait<SelectorType=ParryPairSelector, ShapeCategory=ShapeCategoryOParryShape, OutputCategory=ToParryProximityOutputCategory>,
                                                                                                                                                  Q2: OPairGroupQryTrait<SelectorType=ParryPairSelector, ShapeCategory=ShapeCategoryOParryShape, OutputCategory=ToParryProximityOutputCategory> {
    pub fn new(robot: Arc<ORobot<T, C, L>>, environment: Arc<OParryGenericShapeScene<T, C::P<T>>>, start_state: Vec<T>, boundary_point_a: Vec<T>, boundary_point_b: Vec<T>, self_proximity_query: OwnedPairGroupQry<'a, T, Q1>, environment_proximity_query: OwnedPairGroupQry<'a, T, Q2>, self_selector: ParryPairSelector, environment_selector: ParryPairSelector, distance_cutoff: T, p_norm: T) -> Self {
        Self { robot, environment, start_state, boundary_point_a, boundary_point_b, self_proximity_query, environment_proximity_query, self_selector, environment_selector, distance_cutoff, p_norm }
    }
    pub fn to_other_ad_type<T1: AD>(&self) -> DifferentiableFunctionCollisionStateResolver<'a, T1, C, L, Q1, Q2> {
        DifferentiableFunctionCollisionStateResolver {
            robot: Arc::new(self.robot.to_other_generic_types::<T1, C, L>()),
            environment: Arc::new(self.environment.to_other_generic_types::<T1, C>()),
            start_state: self.start_state.ovec_to_other_ad_type::<T1>(),
            boundary_point_a: self.boundary_point_a.ovec_to_other_ad_type::<T1>(),
            boundary_point_b: self.boundary_point_b.ovec_to_other_ad_type::<T1>(),
            self_proximity_query: self.self_proximity_query.to_other_ad_type::<T1>(),
            environment_proximity_query: self.environment_proximity_query.to_other_ad_type::<T1>(),
            self_selector: self.self_selector.clone(),
            environment_selector: self.environment_selector.clone(),
            distance_cutoff: self.distance_cutoff.to_other_ad_type::<T1>(),
            p_norm: self.p_norm.to_other_ad_type::<T1>()
        }
    }
}
impl<'a, T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static, Q1, Q2> DifferentiableFunctionTrait<'a, T> for DifferentiableFunctionCollisionStateResolver<'a, T, C, L, Q1, Q2>
    where Q1: OPairGroupQryTrait<SelectorType=ParryPairSelector, ShapeCategory=ShapeCategoryOParryShape, OutputCategory=ToParryProximityOutputCategory>,
          Q2: OPairGroupQryTrait<SelectorType=ParryPairSelector, ShapeCategory=ShapeCategoryOParryShape, OutputCategory=ToParryProximityOutputCategory>
{
    fn call(&self, inputs: &[T], freeze: bool) -> Vec<T> {
        let x = inputs.to_vec();

        let mut out = T::zero();

        let fk_res = self.robot.forward_kinematics(&x, None);
        let res1 = self.robot.parry_shape_scene_self_query_from_fk_res(&fk_res, &self.self_proximity_query, &self.self_selector, freeze);
        let res2 = self.robot.parry_shape_scene_external_query_from_fk_res(&fk_res, &self.environment, &self.environment_proximity_query, &self.environment_selector, freeze);

        let p1 = res1.get_proximity_objective_value(self.distance_cutoff, self.p_norm, ProximityLossFunction::Hinge);
        let p2 = res2.get_proximity_objective_value(self.distance_cutoff, self.p_norm, ProximityLossFunction::Hinge);

        let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T::zero(), T::constant(2.0), T::constant(0.3), T::constant(1.0), T::constant(2.0));
        out += loss.loss(p1);
        out += loss.loss(p2);

        let diff = self.start_state.ovec_sub(&x);
        let diff_p = diff.ovec_p_norm(&T::constant(15.0));
        out += T::constant(4.0)*loss.loss(diff_p);

        let b_sub_a = self.boundary_point_b.ovec_sub(&self.boundary_point_a);
        let b_sub_a_normalized = b_sub_a.ovec_scalar_div(&b_sub_a.ovec_p_norm(&T::constant(2.0)));

        let x_sub_a = x.ovec_sub(&self.boundary_point_a);
        let x_sub_a_normalized = x_sub_a.ovec_scalar_div(&x_sub_a.ovec_p_norm(&T::constant(2.0)));

        let b_sub_x = self.boundary_point_b.ovec_sub(&x);
        let b_sub_x_normalized = b_sub_x.ovec_scalar_div(&b_sub_x.ovec_p_norm(&T::constant(2.0)));

        let d1 = b_sub_a_normalized.ovec_dot(&x_sub_a_normalized).powi(2);
        let d2 = b_sub_a_normalized.ovec_dot(&b_sub_x_normalized).powi(2);

        let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T::zero(), T::constant(2.0), T::constant(0.3), T::constant(1.0), T::constant(2.0));
        out += T::constant(1.0)*loss.loss(d1);
        out += T::constant(1.0)*loss.loss(d2);

        return vec![out];
    }

    fn num_inputs(&self) -> usize {
        self.robot.num_dofs()
    }

    fn num_outputs(&self) -> usize {
        1
    }
}

pub type DifferentiableBlockCollisionStateResolver<'a, C, L, Q1, Q2, E> = DifferentiableBlock<'a, DifferentiableFunctionClassCollisionStateResolver<C, L, Q1, Q2>, E>;