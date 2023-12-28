use std::marker::PhantomData;
use ad_trait::AD;
use ad_trait::ADNumMode::{ForwardAD, ReverseAD};
use ad_trait::differentiable_block::DifferentiableBlock2;
use ad_trait::differentiable_function::{DifferentiableFunctionClass, DifferentiableFunctionTrait2, FiniteDifferencing2, ForwardAD2, ForwardADMulti2, ReverseAD2};
use ad_trait::forward_ad::adfn::adfn;
use ad_trait::reverse_ad::adr::adr;
use optima_3d_spatial::optima_3d_pose::O3DPoseCategoryIsometry3;
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_linalg::{OLinalgCategoryNalgebra, OVec};
use optima_proximity::pair_group_queries::{OPairGroupQryTrait, OwnedPairGroupQry, OwnedParryDistanceAsProximityGroupQry, ParryDistanceGroupArgs, ParryPairSelector, ProximityLossFunction, ToParryProximityOutputCategory};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity::proxima::{OwnedParryProximaAsProximityQry, OwnedParryProximaQry, PairGroupQryArgsParryProxima, ParryProximaGroupOutput, ProximaTermination};
use optima_proximity::shapes::ShapeCategoryOParryShape;
use optima_robotics::robot::{ORobot, ORobotDefault, SaveRobot};

pub struct TestFunctionClass<Q>(PhantomData<Q>) where Q: OPairGroupQryTrait<OutputCategory=ToParryProximityOutputCategory, ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector>;
impl<Q> DifferentiableFunctionClass for TestFunctionClass<Q>
    where Q: OPairGroupQryTrait<OutputCategory=ToParryProximityOutputCategory, ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector>
{
    type FunctionType<'a, T: AD> = TestFunction<'a, T, Q>;
}

pub struct TestFunction<'a, T: AD, Q>
    where Q: OPairGroupQryTrait<OutputCategory=ToParryProximityOutputCategory, ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector>
{
    robot: ORobot<T, O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra>,
    q: OwnedPairGroupQry<'a, T, Q>
}
impl<'a, T: AD, Q> DifferentiableFunctionTrait2<'a, T> for TestFunction<'a, T, Q>
    where Q: OPairGroupQryTrait<OutputCategory=ToParryProximityOutputCategory, ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector>
{
    fn call(&self, inputs: &[T], freeze: bool) -> Vec<T> {
        let res = self.robot.parry_shape_scene_self_query(&inputs.to_vec(), &self.q, &ParryPairSelector::HalfPairs, freeze);
        let proximity = res.get_proximity_objective_value(T::constant(0.7), T::constant(15.0), ProximityLossFunction::Hinge);
        vec![proximity]
        // let res = self.robot.forward_kinematics(&inputs.to_vec(), None);
        // let res = res.get_link_pose(6).as_ref().unwrap();
        // let res = res.translation.vector.norm();
        // vec![res]
    }

    fn num_inputs(&self) -> usize {
        self.robot.num_dofs()
    }

    fn num_outputs(&self) -> usize {
        1
    }
}

pub type DifferentiableBlockTestFunction<'a, Q, E> = DifferentiableBlock2<'a, TestFunctionClass<Q>, E>;

fn main() {
    type ADType = adfn<6>;
    let r = ORobotDefault::load_from_saved_robot("ur5");
    let q = OwnedParryProximaAsProximityQry::new(PairGroupQryArgsParryProxima::new(ParryShapeRep::OBB, true, false, ProximaTermination::MaxError(0.25), ProximityLossFunction::Hinge, 15.0, 0.7));
    // let q = OwnedParryDistanceAsProximityGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::OBB, ParryDisMode::ContactDis, true, false, f64::MIN));

    let t2 = TestFunction {
        robot: r.to_other_ad_type::<ADType>(),
        q: q.to_other_ad_type::<ADType>(),
    };
    let t = TestFunction {
        robot: r,
        q,
    };

    let d = DifferentiableBlockTestFunction::new(ForwardADMulti2::new(), t, t2);

    let mut state = vec![3.0; 6];
    for _ in 0..701 {
        println!("{:?}", state);
        let res = d.derivative(&state);
        println!("{:?}", res);
        state = state.ovec_add(&vec![0.01; 6]);
    }

}