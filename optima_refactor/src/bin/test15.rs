use ad_trait::differentiable_function::{FiniteDifferencing, ForwardADMulti};
// use ad_trait::forward_ad::adfn::adfn;
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_interpolation::InterpolatorTrait;
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_optimization::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
// use optima_optimization::nlopt::{Algorithm, NLOptOptimizer};
use optima_optimization::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{OwnedEmptyParryFilter, OwnedEmptyToProximityQry};
use optima_robotics::robot::{ORobotDefault};
use optima_robotics::robotics_optimization::robotics_optimization_functions::{AxisDirection, LookAtTarget};
// use optima_robotics::robotics_optimization::robotics_optimization_look_at::DifferentiableBlockLookAtTrait;


fn main() {
    let r = ORobotDefault::load_from_saved_robot("xarm7_bimanual_viewpoint");
    // r.bevy_display();

    let init_state = vec![0.0,0.0,-1.5,0.0,0.3,0.0,0.24,0.0,0.2,3.7,-1.44,0.0,0.3,0.0,0.2,0.0];

    let fq = OwnedEmptyParryFilter::new(());
    let q = OwnedEmptyToProximityQry::new(());
    let db = r.get_look_at_differentiable_block(FiniteDifferencing::new(), fq, q, None, &init_state, vec![20], 32, AxisDirection::Z, AxisDirection::Y, LookAtTarget::RobotLink(20), 1.2, 0.0, 0.0, 1.0, 0.0, 0.5, 0.3, 0.1, 0.4, 0.0, 0.5);
    let o = SimpleOpEnOptimizer::new(r.get_dof_lower_bounds(), r.get_dof_upper_bounds(), 0.001);
    // let o = NLOptOptimizer::new(Algorithm::Slsqp, false, 16, None, Some(r.get_dof_lower_bounds()), Some(r.get_dof_upper_bounds()));

    let mut solutions = vec![init_state.clone()];
    let mut curr_solution = init_state.clone();
    for i in 0..1000 {
        let solution = o.optimize_unconstrained(&curr_solution, &db);
        curr_solution = solution.x_star().to_vec();
        solutions.push(curr_solution.clone());
        println!("{:?}", curr_solution);
        // db.update_goal_distance_between_looker_and_look_at_target(1.1 + (i as f64) * 0.001);
    }

    let interpolator = InterpolatingSpline::new(solutions, InterpolatingSplineType::Linear).to_timed_interpolator(6.0);
    r.bevy_motion_playback(&interpolator);
}