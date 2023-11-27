use ad_trait::differentiable_function::{ForwardADMulti};
use ad_trait::forward_ad::adf::adf_f32x8;
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_optimization::{DiffBlockUnconstrainedOptimizerTrait, OptimizerOutputTrait};
use optima_optimization::optimization_engine::SimpleOpEnEngineOptimizer;
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_diffblock_spawners::RoboticsDiffBlockSpawners;
use optima_robotics::robotics_optimization_solvers::{DifferentiableBlockIKObjectiveTrait};
use optima_robotics::robotics_optimization::robotics_optimization_utils::RoboticsOptimizationUtils;


type DerivativeMethod = ForwardADMulti<adf_f32x8>;
fn main() {
    let robot = ORobotDefault::load_from_saved_robot("ur5");
    let mut diff_block = robot.spawn_ik_differentiable_block::<DerivativeMethod>(());
    diff_block.add_initial_ik_goals(vec![6]);
    diff_block.update_ik_goal_pose(0, &Isometry3::from_constructors(&[0.06,0.1,0.5], &[0.,0.,0.]));
    let s = SimpleOpEnEngineOptimizer::new(robot.get_dof_lower_bounds(), robot.get_dof_upper_bounds(), 0.0001);
    let init = robot.sample_pseudorandom_state();
    let res = s.diff_block_unconstrained_optimize(&diff_block, &init);
    println!("{:?}", res.x_star());
    println!("{:?}", res.f_star());
    println!("{:?}", res.solver_status());
}