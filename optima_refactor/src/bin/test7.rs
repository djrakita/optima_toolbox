use ad_trait::differentiable_function::{FiniteDifferencing2, ForwardAD2, ForwardADMulti2, ReverseAD2};
use ad_trait::forward_ad::adf::adf_f64x8;
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_interpolation::InterpolatorTrait;
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_optimization2::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization2::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{EmptyParryFilter, OwnedEmptyParryFilter, OwnedEmptyToProximityQry, OwnedParryDistanceAsProximityGroupQry, OwnedParryDistanceGroupSequenceFilter, OParryDistanceGroupArgs, OParryDistanceGroupSequenceFilterArgs, OParryPairSelector, OProximityLossFunction};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity::proxima::{OwnedParryProximaAsProximityQry, OParryProximaArgs, OProximaTermination};
use optima_robotics::robot::{ORobotDefault};
use optima_robotics::robotics_optimization::robotics_optimization_ik::{DifferentiableBlockIKObjectiveTrait, IKGoalUpdateMode};

fn main() {
    let mut r = ORobotDefault::load_from_saved_robot("xarm7_with_gripper_and_rail_8dof");
    // r.set_joint_as_fixed(12, &[0.0]);
    // r.save_robot(Some("xarm7_with_gripper_and_rail_8dof"));

    let init_condition = vec![0.1; 8];
    let fq = OwnedEmptyParryFilter::new(());
    // let fq = OwnedParryDistanceGroupSequenceFilter::new(ParryDistanceGroupSequenceFilterArgs::new(vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB, ParryShapeRep::Full], vec![], 0.6, true, ParryDisMode::ContactDis));
    // let q = OwnedParryProximaAsProximityQry::new(PairGroupQryArgsParryProxima::new(ParryShapeRep::Full, true, false, ProximaTermination::MaxError(0.2), ProximityLossFunction::Hinge, 15.0, 0.6));
    let q = OwnedParryDistanceAsProximityGroupQry::new(OParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, true, false, -1000.0, false));
    let db = r.get_ik_differentiable_block(ForwardADMulti2::<adfn<8>>::new(), fq, q, None, &init_condition, vec![19], 0.09, 0.6, 1.0, 0.1, 1.0, 0.3, 0.1);
    let o = SimpleOpEnOptimizer::new(r.get_dof_lower_bounds(), r.get_dof_upper_bounds(), 0.001);

    let mut solutions = vec![];
    let mut curr_solution = init_condition.clone();
    for _ in 0..6000 {
        solutions.push(curr_solution.clone());
        let solution = o.optimize_unconstrained(&curr_solution, &db);
        println!("{:?}", solution.solver_status().solve_time());
        // println!("{:?}", solution.solver_status().iterations());
        curr_solution = solution.x_star().to_vec();
        db.update_prev_states(curr_solution.clone());
        db.update_ik_pose(0, Isometry3::from_constructors(&[0.,0.0001,0.0001], &[0.,0.,0.]), IKGoalUpdateMode::GlobalRelative);
    }
    let spline = InterpolatingSpline::new(solutions, InterpolatingSplineType::Linear).to_timed_interpolator(6.0);
    r.bevy_motion_playback(&spline);
}