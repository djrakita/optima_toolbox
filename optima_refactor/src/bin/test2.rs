use ad_trait::differentiable_function::{FiniteDifferencing2, ForwardAD2, ForwardADMulti2, ReverseAD2};
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_interpolation::InterpolatorTrait;
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_optimization2::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization2::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{OwnedEmptyParryFilter, OwnedEmptyToProximityQry, OwnedParryDistanceAsProximityGroupQry, OwnedParryDistanceGroupFilter, OwnedParryDistanceGroupSequenceFilter, OParryDistanceGroupArgs, OParryDistanceGroupFilterArgs, OParryDistanceGroupSequenceFilterArgs, OParryPairSelector, OProximityLossFunction};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity::proxima::{OwnedParryProximaAsProximityQry, OParryProximaArgs, OProximaTermination};
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_optimization::robotics_optimization_functions::{AxisDirection, LookAtTarget};
use optima_robotics::robotics_optimization::robotics_optimization_ik::IKGoalUpdateMode;
use optima_robotics::robotics_optimization::robotics_optimization_look_at::DifferentiableBlockLookAtTrait;

fn main() {
    //robot.save_robot(Some("xarm7_bimanual_viewpoint"));
    let robot = ORobotDefault::load_from_saved_robot("xarm7_bimanual_viewpoint");

    let init = vec![0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3,  0.0, 3.14, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,];
    let qf = OwnedParryDistanceGroupSequenceFilter::new(OParryDistanceGroupSequenceFilterArgs::new(vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB, ParryShapeRep::Full], vec![], 0.6, true, ParryDisMode::ContactDis));
    // let qf = OwnedEmptyParryFilter::new(());
    let q = OwnedParryProximaAsProximityQry::new(OParryProximaArgs::new(ParryShapeRep::Full, true, false, OProximaTermination::MaxError(0.2), OProximityLossFunction::Hinge, 15.0, 0.6));
    // let q = OwnedParryDistanceAsProximityGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, true, false, 0.0, false));
    // let q = OwnedEmptyToProximityQry::new(());
    let db = robot.get_look_at_differentiable_block(ForwardADMulti2::<adfn<16>>::new(), qf, q, None, &init, vec![20], 32, AxisDirection::Z, AxisDirection::X, LookAtTarget::RobotLink(20), 0.09, 0.6, 0.5, 0.1, 1.0, 0.5, 0.1, 0.5, 0.1);
    let o = SimpleOpEnOptimizer::new(robot.get_dof_lower_bounds(), robot.get_dof_upper_bounds(), 0.001);

    let mut states = vec![];
    let mut curr_solution = init.clone();
    for _ in 0..6000 {
        let res = o.optimize_unconstrained(curr_solution.as_slice(), &db);
        states.push(res.x_star().to_vec());
        curr_solution = res.x_star().to_vec();
        println!("{:?}", curr_solution);
        println!("{:?}", res.solver_status().solve_time());

        db.update_prev_states(res.x_star().to_vec());
        db.update_ik_pose(0, Isometry3::from_constructors(&[0.,0.,0.0001], &[0.,0.,0.]), IKGoalUpdateMode::GlobalRelative);
    }
    let spline = InterpolatingSpline::new(states, InterpolatingSplineType::Linear).to_timed_interpolator(10.0);
    robot.bevy_motion_playback(&spline);
}