use ad_trait::differentiable_function::{DerivativeMethodTrait, ForwardAD, ForwardADMulti};
use ad_trait::forward_ad::adf::adf_f32x8;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_optimization::{DiffBlockUnconstrainedOptimizerTrait, OptimizerOutputTrait};
use optima_optimization::optimization_engine::SimpleOpEnEngineOptimizer;
use optima_robotics::robotics_diffblock_spawners::RoboticsDiffBlockSpawners;
use optima_robotics::robotics_optimization_solvers::IKGoal;
use optima_robotics::robotics_optimization_utils::RoboticsOptimizationUtils;
use num_traits::identities::Zero;
use num_traits::One;

type Deriv = ForwardADMulti<adf_f32x8>;
type T = adf_f32x8;
fn main() {
    let robots = RoboticsOptimizationUtils::get_f64_and_ad_default_robots::<Deriv>("ur5");
    let mut diff_block = RoboticsDiffBlockSpawners::spawn_ik_diff_block::<Deriv, _, _>(&robots, ());
    diff_block.update_args(|x, y| {
        x.goals.push(IKGoal {
            goal_link_idx: 6,
            goal_pose: Isometry3::from_constructors(&[0.1, 0.1, 0.6], &[0., 0., 0.]),
            weight: 1.0,
        });

        y.goals.push(IKGoal {
            goal_link_idx: 6,
            goal_pose: Isometry3::from_constructors(&[T::constant(0.1), T::constant(0.1), T::constant(0.6)], &[T::zero(), T::zero(), T::zero()]),
            weight: T::one(),
        });
    });
    let s = SimpleOpEnEngineOptimizer::new(robots.0.get_dof_lower_bounds(), robots.0.get_dof_upper_bounds(), 0.0001);
    let init = robots.0.sample_pseudorandom_state();
    let res = s.diff_block_unconstrained_optimize(&diff_block, &init);
    println!("{:?}", res.x_star());
    println!("{:?}", res.f_star());
    println!("{:?}", res.solver_status());
}