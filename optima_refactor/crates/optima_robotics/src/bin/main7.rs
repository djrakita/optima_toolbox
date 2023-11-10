use std::sync::Mutex;
use ad_trait::differentiable_function::ForwardADMulti;
use ad_trait::forward_ad::adf::adf_f32x8;
use nalgebra::Isometry3;
use optimization_engine::panoc::PANOCCache;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_optimization::optimization_engine::SimpleOpEnEngine;
use optima_optimization::OptimizerTrait;
use optima_robotics::robot::ORobotDefault;
use optima_robotics::robotics_optimization_solvers::{ADTraitBasedIKArgs, ADTraitBasedIKObjective, IKGoal};

type A = adf_f32x8;
fn main() {
    let r1 = ORobotDefault::load_from_saved_robot("ur5");
    let r2 = r1.to_new_ad_type::<A>();

    let p1 = Isometry3::from_constructors(&[0.2,0.,0.6], &[0.,0.,0.]);
    let p2 = p1.to_other_ad_type::<A>();

    let s = SimpleOpEnEngine::<ADTraitBasedIKObjective<_, _>, ForwardADMulti<A>> {
        call_args: ADTraitBasedIKArgs { robot: &r1, goals: vec![IKGoal {
            goal_link_idx: 6,
            goal_pose: p1,
            weight: 1.0,
        }] },
        derivative_args: ADTraitBasedIKArgs {
            robot: &r2,
            goals: vec![ IKGoal {
                goal_link_idx: 6,
                goal_pose: p2,
                weight: A::constant(1.0),
            } ],
        },
        derivative_method_data: (),
        initial_condition: vec![0.1; 6],
        lower_bounds: r1.get_dof_lower_bounds(),
        upper_bounds: r1.get_dof_upper_bounds(),
        cache: Mutex::new(PANOCCache::new(r1.num_dofs(), 0.001, 3)),
    };

    let res = s.optimize();
    println!("{:?}", res.solver_status());
}