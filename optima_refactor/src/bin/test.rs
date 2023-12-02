
use ad_trait::differentiable_function::{FiniteDifferencing2, ForwardAD2, ForwardADMulti2, ReverseAD2};
use ad_trait::forward_ad::adf::{adf_f32x16, adf_f32x8, adf_f64x8};
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_interpolation::InterpolatorTrait;
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_optimization2::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization2::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{OwnedParryDistanceGroupQry, OwnedParryDistanceGroupSequenceFilter, ParryDistanceGroupArgs, ParryDistanceGroupSequenceFilterArgs};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_optimization2::robotics_optimization_ik::{DifferentiableBlockIKObjectiveTrait, IKGoalUpdateMode};

fn main() {
    let r = ORobotDefault::load_from_saved_robot("ur5");

    let distance_threshold = 0.6;
    let fq = OwnedParryDistanceGroupSequenceFilter::new(ParryDistanceGroupSequenceFilterArgs::new(vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB], vec![ParryShapeRep::BoundingSphere, ParryShapeRep::OBB], 0.6, true, ParryDisMode::ContactDis));
    let q = OwnedParryDistanceGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, true, f64::MIN));

    let init_state = [0.6; 6];
    let mut db1 = r.get_ik_differentiable_block(ForwardADMulti2::<adf_f64x8>::new(), fq, q, &init_state, vec![6], distance_threshold);

    let o = SimpleOpEnOptimizer::new(r.get_dof_lower_bounds(), r.get_dof_upper_bounds(), 0.001);

    let mut solution = init_state.to_vec();
    let mut states = vec![];
    for _ in 0..100 {
        states.push(solution.clone());
        let res = o.optimize_unconstrained(&solution, &db1);
        println!("{:?}, {:?}", res.x_star(), res.f_star());
        solution = res.x_star().to_vec();
        db1.update_ik_pose(0, Isometry3::from_constructors(&[-0.008, -0.008, -0.00], &[0.; 3]), IKGoalUpdateMode::GlobalRelative);
        db1.update_prev_states(solution.clone());
    }

    let spline = InterpolatingSpline::new(states, InterpolatingSplineType::Linear).to_timed_interpolator(6.0);
    r.bevy_motion_playback(&spline);
}
