use ad_trait::differentiable_function::{FiniteDifferencing2, ForwardAD2, ReverseAD2};
use optima_optimization2::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization2::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{OwnedEmptyParryFilter, OwnedEmptyToProximityQry, OwnedParryDistanceAsProximityGroupQry, ParryDistanceGroupArgs, ParryPairSelector};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity::proxima::{OwnedParryProximaAsProximityQry, PairGroupQryArgsParryProxima, ProximaTermination};
use optima_robotics::robot::ORobotDefault;

fn main() {
    // let q = OwnedParryDistanceAsProximityGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, true, false, f64::MIN));
    // let q = OwnedParryProximaAsProximityQry::new(PairGroupQryArgsParryProxima::new(ParryShapeRep::Full, true, false, ProximaTermination::MaxError(0.3), 15.0, 0.7));
    let q = OwnedEmptyToProximityQry::new(());
    let r = ORobotDefault::load_from_saved_robot("ur5");

    let ik = r.get_ik_differentiable_block(ForwardAD2::new(), OwnedEmptyParryFilter::new(()), q, Some(ParryPairSelector::HalfPairs),&[0.0; 6], vec![6], 0.07, 0.7, 1.0, 1.0, 1.0, 1.0, 1.0);

    let o = SimpleOpEnOptimizer::new(r.get_dof_lower_bounds(), r.get_dof_upper_bounds(), 0.001);
    let res = o.optimize_unconstrained(&[0.01; 6], &ik);
    println!("{:?}", res.x_star());
}