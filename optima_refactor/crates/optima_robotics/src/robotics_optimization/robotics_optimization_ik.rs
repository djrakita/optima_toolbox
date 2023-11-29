use std::borrow::Cow;
use std::marker::PhantomData;
use std::sync::RwLock;
use ad_trait::AD;
use ad_trait::differentiable_block::DifferentiableBlockArgPrepTrait;
use ad_trait::differentiable_function::{DerivativeMethodTrait, DifferentiableFunctionTrait};
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryTrait};
use optima_linalg::{OLinalgCategoryTrait, OVec, OVecCategoryVec};
use optima_optimization::loss_functions::{GrooveLossGaussianDirection, OptimizationLossFunctionTrait, OptimizationLossGroove};
use optima_proximity::pair_group_queries::{OPairGroupQryTrait, OwnedPairGroupQry, PairGroupQryOutputCategoryParryDistance, PairGroupQryOutputCategoryParryFilter, ParryFilterOutput, ParryPairSelector, ProximityLossFunctionHinge, ToParryProximityOutputTrait};
use optima_proximity::shapes::ShapeCategoryOParryShape;
use crate::robot::ORobot;

pub struct IKObjective<C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static, FQ, Q>(PhantomData<(C, L, FQ, Q)>) where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
                                                                                                                                      Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance>;
impl<C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static, FQ, Q> DifferentiableFunctionTrait for IKObjective<C, L, FQ, Q> where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
                                                                                                                                                 Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance> {
    type ArgsType<'a, T: AD> = IKArgs<'a, T, C, L, FQ, Q>;

    fn call<'a, T1: AD>(inputs: &[T1], args: &Self::ArgsType<'a, T1>) -> Vec<T1> {
        let fk_res = args.robot.forward_kinematics(&inputs.to_vec(), None);

        let mut out = T1::zero();
        args.goals.iter().for_each(|g| {
            let pose = fk_res.get_link_pose(g.goal_link_idx).as_ref().expect("error");
            let dis = pose.dis(&g.goal_pose);
            let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T1::zero(), T1::constant(2.0), T1::constant(0.1), T1::constant(1.0), T1::constant(2.0));
            out += g.weight * loss.loss(dis);
        });

        let binding = args.filter_output.read();
        let filter_output = binding.as_ref().unwrap();
        let distance_output = match &**filter_output {
            None => {
                args.robot.parry_shape_scene_self_query(&inputs.to_vec(), &args.distance_query, &ParryPairSelector::HalfPairs)
            }
            Some(filter_output) => {
                args.robot.parry_shape_scene_self_query(&inputs.to_vec(), &args.distance_query, filter_output.selector())
            }
        };

        if args.collision_avoidance_weight > T1::zero() {
            let proximity_value = distance_output.compute_proximity_objective_value(T1::constant(0.6), T1::constant(10.0), ProximityLossFunctionHinge {});
            let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T1::zero(), T1::constant(6.0), T1::constant(0.4), T1::constant(2.0), T1::constant(4.0));
            out += args.collision_avoidance_weight*loss.loss(proximity_value);
        }

        let x_vec = inputs.to_vec();
        let v0 = x_vec.ovec_sub(&args.prev_states.prev_state_0);
        let v1 = args.prev_states.prev_state_0.ovec_sub(&args.prev_states.prev_state_1);
        let v2 = args.prev_states.prev_state_1.ovec_sub(&args.prev_states.prev_state_2);
        let a0 = v0.ovec_sub(&v1);
        let a1 = v1.ovec_sub(&v2);
        let j0 = a0.ovec_sub(&a1);

        let loss = OptimizationLossGroove::new(GrooveLossGaussianDirection::BowlUp, T1::zero(), T1::constant(2.0), T1::constant(0.2), T1::constant(2.0), T1::constant(2.0));
        let v = v0.ovec_p_norm(&T1::constant(12.0));
        let a = a0.ovec_p_norm(&T1::constant(12.0));
        let j = j0.ovec_p_norm(&T1::constant(12.0));

        out += args.min_vel_weight*loss.loss(v);
        out += args.min_acc_weight*loss.loss(a);
        out += args.min_jerk_weight*loss.loss(j);

        vec![out]
    }

    fn num_inputs<T1: AD>(args: &Self::ArgsType<'_, T1>) -> usize {
        args.robot.num_dofs()
    }

    fn num_outputs<T1: AD>(_args: &Self::ArgsType<'_, T1>) -> usize {
        1
    }
}

pub struct IKArgs<'a, T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static, FQ, Q> where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
                                                                                                                Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance> {
    pub robot: Cow<'a, ORobot<T, C, L>>,
    pub goals: Vec<IKGoal<T, C::P<T>>>,
    pub filter_output: RwLock<Option<ParryFilterOutput>>,
    pub filter_query: OwnedPairGroupQry<'a, T, C::P<T>, FQ>,
    pub distance_query: OwnedPairGroupQry<'a, T, C::P<T>, Q>,
    pub prev_states: PrevStates<T>,
    pub ee_matching_weight: T,
    pub collision_avoidance_weight: T,
    pub min_vel_weight: T,
    pub min_acc_weight: T,
    pub min_jerk_weight: T
}
impl<'a, T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static, FQ, Q> IKArgs<'a, T, C, L, FQ, Q> where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
                                                                                                                              Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance> {
    pub fn new(robot: Cow<'a, ORobot<T, C, L>>, goals: Vec<IKGoal<T, C::P<T>>>, filter_query: OwnedPairGroupQry<'a, T, C::P<T>, FQ>, distance_query: OwnedPairGroupQry<'a, T, C::P<T>, Q>, init_state: Vec<T>, ee_matching_weight: T, collision_avoidance_weight: T, min_vel_weight: T, min_acc_weight: T, min_jerk_weight: T) -> Self {
        assert!(ee_matching_weight >= T::zero());
        assert!(collision_avoidance_weight >= T::zero());
        assert!(min_vel_weight >= T::zero());
        assert!(min_acc_weight >= T::zero());
        assert!(min_jerk_weight >= T::zero());

        Self { robot, goals, filter_output: RwLock::new(None), filter_query, distance_query, prev_states: PrevStates::new(init_state), ee_matching_weight, collision_avoidance_weight, min_vel_weight, min_acc_weight, min_jerk_weight }
    }
}

pub struct IKArgsPrep<C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static, FQ, Q> where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
                                                                                                         Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance> {
    last_proximity_filter_state: RwLock<Option<Vec<f64>>>,
    linf_norm_cutoff: f64,
    phantom_data: PhantomData<(C, L, FQ, Q)>
}
impl<C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static, FQ, Q> IKArgsPrep<C, L, FQ, Q> where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
                                                                                                                Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance> {
    pub fn new(linf_norm_cutoff: f64) -> Self {
        Self { last_proximity_filter_state: RwLock::new(None), linf_norm_cutoff, phantom_data: PhantomData::default() }
    }
}
impl<'a, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static, FQ, Q, E: DerivativeMethodTrait> DifferentiableBlockArgPrepTrait<'a, IKObjective<C, L, FQ, Q>, E> for IKArgsPrep<C, L, FQ, Q> where FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
                                                                                                                                                                                                                   Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryDistance> {
    fn prep_args(&self, inputs: &[f64], function_standard_args: &<IKObjective<C, L, FQ, Q> as DifferentiableFunctionTrait>::ArgsType<'a, f64>, function_derivative_args: &<IKObjective<C, L, FQ, Q> as DifferentiableFunctionTrait>::ArgsType<'a, E::T>) {
        if function_standard_args.collision_avoidance_weight > 0.0 {
            let mut trigger_proximity_filter = false;
            let inputs_vec = inputs.to_vec();
            let binding = self.last_proximity_filter_state.read();
            let last_proximity_filter_state_read = &**binding.as_ref().unwrap();
            match last_proximity_filter_state_read {
                None => { trigger_proximity_filter = true }
                Some(last_proximity_filter_state_read) => {
                    let linf = last_proximity_filter_state_read.ovec_sub(&inputs_vec).ovec_linf_norm();
                    if linf > self.linf_norm_cutoff { trigger_proximity_filter = true; }
                }
            }
            drop(binding);

            if trigger_proximity_filter {
                let binding = self.last_proximity_filter_state.write();
                let mut last_proximity_filter_state_write = binding.unwrap();
                *last_proximity_filter_state_write = Some(inputs_vec.clone());

                let filter_result = function_standard_args.robot.parry_shape_scene_self_query(&inputs_vec, &function_standard_args.filter_query, &ParryPairSelector::HalfPairs);
                *function_standard_args.filter_output.write().unwrap() = Some(filter_result.clone());
                *function_derivative_args.filter_output.write().unwrap() = Some(filter_result);
            }
        }
    }
}

#[derive(Clone)]
pub struct IKGoal<T: AD, P: O3DPose<T>> {
    pub goal_link_idx: usize,
    pub goal_pose: P,
    pub weight: T,
}

#[derive(Clone, Debug)]
pub struct PrevStates<T: AD> {
    prev_state_0: Vec<T>,
    prev_state_1: Vec<T>,
    prev_state_2: Vec<T>,
    prev_state_3: Vec<T>,
}
impl<T: AD> PrevStates<T> {
    pub fn new(init_state: Vec<T>) -> Self {
        Self {
            prev_state_0: init_state.clone(),
            prev_state_1: init_state.clone(),
            prev_state_2: init_state.clone(),
            prev_state_3: init_state,
        }
    }
    #[inline(always)]
    pub fn update<T1: AD, V: OVec<T1>>(&mut self, new_state: V) {
        let new_state = new_state.ovec_to_other_generic_category::<T, OVecCategoryVec>();
        self.update_internal(new_state);
    }
    #[inline(always)]
    fn update_internal(&mut self, new_state: Vec<T>) {
        self.prev_state_3 = self.prev_state_2.to_owned();
        self.prev_state_2 = self.prev_state_1.to_owned();
        self.prev_state_1 = self.prev_state_0.to_owned();
        self.prev_state_0 = new_state;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
use std::marker::PhantomData;
use ad_trait::AD;
use ad_trait::differentiable_function::{DifferentiableFunctionTrait};
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryTrait};
use optima_linalg::OLinalgCategoryTrait;
use optima_optimization::derivative_based_optimization::{DerivBasedOptSolver};
use crate::robot::ORobot;

pub struct SimpleIKArgs<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> {
    robot: ORobot<T, C, L>,
    pub chain_idx: usize,
    pub link_idx: usize,
    pub goal_pose: C::P<T>
}
impl<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> SimpleIKArgs<T, C, L> {
    pub fn new(robot: ORobot<T, C, L>, chain_idx: usize, link_idx: usize, goal_pose: C::P<T>) -> Self {
        Self {
            robot,
            chain_idx,
            link_idx,
            goal_pose,
        }
    }
    pub fn robot(&self) -> &ORobot<T, C, L> {
        &self.robot
    }
}

pub struct SimpleIKFunction<C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait>(PhantomData<(C, L)>);
impl<C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> DifferentiableFunctionTrait for SimpleIKFunction<C, L> {
    type ArgsType<T: AD> = SimpleIKArgs<T, C, L>;

    fn call<T1: AD>(inputs: &[T1], args: &Self::ArgsType<T1>) -> Vec<T1> {
        let fk_res = args.robot.forward_kinematics(&inputs, None);
        let pose = fk_res.get_chain_fk_result(args.chain_idx).as_ref().expect("error").get_link_pose(args.link_idx).as_ref().expect("error");
        let dis = pose.dis(&args.goal_pose);

        return vec![dis.powi(2)]
    }

    fn num_inputs<T1: AD>(args: &Self::ArgsType<T1>) -> usize {
        args.robot.num_dofs()
    }

    fn num_outputs<T1: AD>(_args: &Self::ArgsType<T1>) -> usize {
        1
    }
}

pub type SimpleIKSolver<C, L, E, O> = DerivBasedOptSolver<SimpleIKFunction<C, L>, E, O>;
*/