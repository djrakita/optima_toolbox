use ad_trait::differentiable_function::{FiniteDifferencing2, ForwardADMulti2};
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_interpolation::InterpolatorTrait;
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_optimization2::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization2::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{OwnedEmptyParryFilter, OwnedEmptyToProximityQry};
use optima_robotics::robot::{IKGoalMode, ORobotDefault};
use optima_robotics::robotics_optimization::robotics_optimization_ik::{DifferentiableBlockIKObjectiveTrait, IKGoalUpdateMode};

fn main() {
    let r = ORobotDefault::load_from_saved_robot("ur5");

    let init_condition = vec![0.001,0.001,2.0,0.001,0.001,0.001];
    let ik_goal = r.get_ik_goal(&init_condition, 9, IKGoalMode::GlobalRelativeSeparate { offset: Isometry3::from_constructors(&[0.,0.,-0.5], &[0.,0.,0.]) });
    println!("{:?}", ik_goal);
    let db = r.get_ik_differentiable_block(ForwardADMulti2::<adfn<6>>::new(), OwnedEmptyParryFilter::new(()), OwnedEmptyToProximityQry::new(()), None, &init_condition, vec![9], 0.0, 0.0,  1.0, 0.0, 1.0, 0.5, 0.2);

    let o = SimpleOpEnOptimizer::new(r.get_dof_lower_bounds(), r.get_dof_upper_bounds(), 0.001);

    let mut solutions = vec![init_condition.clone()];
    let mut curr_solution = init_condition.clone();
    for _ in 0..60000 {
        let fk_res = r.forward_kinematics(&curr_solution, None);
        let pose = fk_res.get_link_pose(9).as_ref().expect("error");
        let interpolated_goal = pose.interpolate_with_separate_max_translation_and_rotation(&ik_goal, 0.001, 0.0001);
        db.update_ik_pose(0, interpolated_goal, IKGoalUpdateMode::Absolute);

        let solution = o.optimize_unconstrained(&curr_solution, &db);
        // println!("{:?}", solution.solver_status().solve_time());
        // println!("{:?}", solution.f_star());
        curr_solution = solution.x_star().to_vec();
        solutions.push(curr_solution.clone());
        db.update_prev_states(curr_solution.clone());
    }

    let s = InterpolatingSpline::new(solutions, InterpolatingSplineType::Linear).to_timed_interpolator(10.0);
    r.bevy_motion_playback(&s);
}