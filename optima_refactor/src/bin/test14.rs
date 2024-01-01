use ad_trait::differentiable_function::ForwardADMulti;
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_interpolation::InterpolatorTrait;
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_optimization::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{OwnedEmptyParryFilter, OwnedEmptyToProximityQry};
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_optimization::robotics_optimization_ik::{DifferentiableBlockIKObjectiveTrait, IKGoalUpdateMode};

fn main() {
    let robot = ORobotDefault::load_from_saved_robot("ur5");
    let db = robot.get_ik_differentiable_block(ForwardADMulti::<adfn<7>>::new(), OwnedEmptyParryFilter::new(()), OwnedEmptyToProximityQry::new(()), None, &[0.1; 6], vec![9], 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
    let o = SimpleOpEnOptimizer::new(robot.get_dof_lower_bounds(), robot.get_dof_upper_bounds(), 0.001);

    db.update_ik_pose(0, Isometry3::from_constructors(&[0.5,0.5,0.5], &[0.,0.,0.]), IKGoalUpdateMode::Absolute);

    let res = o.optimize_unconstrained(&[0.1; 6], &db);
    let solution = res.x_star().to_vec();
    // println!("{:?}", solution);

    let spline = InterpolatingSpline::new(vec![vec![0.1; 6], solution.clone()], InterpolatingSplineType::Linear)
        .to_arclength_parameterized_interpolator(40);

    let path = spline.interpolate_points_by_arclength_absolute_stride(0.01);
    println!("{:?}", path.len());
}