use ad_trait::differentiable_function::FiniteDifferencing;
use optima_optimization::optimization_engine::{SimpleOpEnEngineConstructor, SimpleOpEnEngineConstructorArgs};
use optima_robotics::robot::ORobotDefault;

fn main() {
    let r = ORobotDefault::from_urdf("ur5");
    let mut ik = r.get_ik_solver::<FiniteDifferencing, SimpleOpEnEngineConstructor>((), &SimpleOpEnEngineConstructorArgs { tolerance: 0.001 });

    /*
    let r1 = ORobotDefault::load_from_saved_robot("ur5");
    let r2 = r1.to_new_ad_type::<A>();

    let p1 = Isometry3::from_constructors(&[0.2,0.,0.6], &[0.,0.,0.]);
    let p2 = p1.to_other_ad_type::<A>();

    let s = SimpleOpEnEngineOptimizer::<IKObjective<_, _>, ForwardADMulti<A>> {
        call_args: IKArgs { robot: &r1, goals: vec![IKGoal {
            goal_link_idx: 6,
            goal_pose: p1,
            weight: 1.0,
        }] },
        derivative_args: IKArgs {
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
    */
}