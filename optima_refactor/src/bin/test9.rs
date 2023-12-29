use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_robotics::robot::ORobotDefault;

fn main() {
    let robot = ORobotDefault::load_from_saved_robot("xarm7_with_gripper_and_rail_8dof");
    robot.bevy_display();

    /*
    // let fq = OwnedParryDistanceGroupSequenceFilter::new(ParryDistanceGroupSequenceFilterArgs::new(vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB, ParryShapeRep::Full], vec![], 0.6, true, ParryDisMode::ContactDis));
    let fq = OwnedEmptyParryFilter::new(());
    // let q = OwnedParryProximaAsProximityQry::new(PairGroupQryArgsParryProxima::new(ParryShapeRep::Full, true, false, ProximaTermination::MaxError(0.1), ProximityLossFunction::Hinge, 15.0, 0.6));
    let q = OwnedEmptyToProximityQry::new(());
    let initial_state = vec![0.3; 8];
    let db = robot.get_ik_differentiable_block(ForwardADMulti2::<adfn<8>>::new(), fq, q, None, &initial_state, vec![19], 0.09, 0.6, 1.0, 0.0, 0.0, 0.0, 0.0);
    let o = SimpleOpEnOptimizer::new(robot.get_dof_lower_bounds(), robot.get_dof_upper_bounds(), 0.001);

    let mut solutions = vec![];
    let mut curve = vec![];
    let mut curr_solution = initial_state.clone();
    for _ in 0..6000 {
        solutions.push(curr_solution.clone());
        let ik_goal_binding = db.function_standard().ik_goals().read().unwrap();
        let ik_goal_pose = ik_goal_binding.get_ik_goal_pose(0);
        curve.push(ik_goal_pose.translation.vector.clone());
        drop(ik_goal_binding);
        let res = o.optimize_unconstrained(&curr_solution, &db);
        println!("{:?}", res.solver_status().solve_time());
        curr_solution = res.x_star().to_vec();
        db.update_ik_pose(0, Isometry3::from_constructors(&[0.0, 0.0, 0.0001], &[0.,0.,0.]), IKGoalUpdateMode::GlobalRelative);
        db.update_prev_states(curr_solution.clone());
    }

    let s = InterpolatingSpline::new(solutions, InterpolatingSplineType::Linear).to_timed_interpolator(6.0);
    let c = InterpolatingSpline::new(curve, InterpolatingSplineType::Linear);
    let mut app = robot.bevy_get_motion_playback_app(&s);
    app.optima_bevy_draw_3d_curve(c, 100, 10.0, 5, 2);
    app.run();

     */
}