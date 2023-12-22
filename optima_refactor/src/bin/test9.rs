use ad_trait::differentiable_function::{FiniteDifferencing2, ForwardADMulti2, ReverseAD2};
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_interpolation::InterpolatorTrait;
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_optimization2::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization2::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{OwnedParryDistanceGroupSequenceFilter, ParryDistanceGroupSequenceFilterArgs, ProximityLossFunction};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity::proxima::{OwnedParryProximaAsProximityQry, PairGroupQryArgsParryProxima, ProximaTermination};
use optima_robotics::robot::{ORobotDefault};
use optima_robotics::robotics_optimization2::robotics_optimization_ik::{DifferentiableBlockIKObjectiveTrait, IKGoalUpdateMode};

fn main() {
    let robot = ORobotDefault::load_from_saved_robot("xarm7_with_gripper_and_rail_8dof");

    let fq = OwnedParryDistanceGroupSequenceFilter::new(ParryDistanceGroupSequenceFilterArgs::new(vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB, ParryShapeRep::Full], vec![], 0.6, true, ParryDisMode::ContactDis));
    let q = OwnedParryProximaAsProximityQry::new(PairGroupQryArgsParryProxima::new(ParryShapeRep::Full, true, false, ProximaTermination::MaxError(0.1), ProximityLossFunction::Hinge, 15.0, 0.6));
    let initial_state = vec![0.3; 8];
    let db = robot.get_ik_differentiable_block(ForwardADMulti2::<adfn<8>>::new(), fq, q, None, &initial_state, vec![19], 0.09, 0.6, 1.0, 0.2, 1.0, 0.3, 0.1);
    let o = SimpleOpEnOptimizer::new(robot.get_dof_lower_bounds(), robot.get_dof_upper_bounds(), 0.001);

    let mut solutions = vec![];
    let mut curr_solution = initial_state.clone();
    for _ in 0..3000 {
        solutions.push(curr_solution.clone());
        let res = o.optimize_unconstrained(&curr_solution, &db);
        println!("{:?}", res.solver_status().solve_time());
        curr_solution = res.x_star().to_vec();
        db.update_ik_pose(0, Isometry3::from_constructors(&[0.0, 0.0, 0.0001], &[0.,0.,0.]), IKGoalUpdateMode::GlobalRelative);
        db.update_prev_states(curr_solution.clone());
    }

    let s = InterpolatingSpline::new(solutions, InterpolatingSplineType::Linear).to_timed_interpolator(6.0);
    robot.bevy_motion_playback(&s);
}