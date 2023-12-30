use std::sync::Arc;
use std::time::Instant;
use ad_trait::differentiable_function::{FiniteDifferencing, ForwardADMulti};
use ad_trait::forward_ad::adfn::adfn;
use nalgebra::{Isometry3, Vector3};
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_3d_spatial::optima_3d_rotation::QuatConstructor;
use optima_bevy::{App, OptimaBevyTrait};
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_interpolation::{get_interpolation_range_num_steps, InterpolatorTrait, InterpolatorTraitLite};
use optima_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use optima_linalg::OVec;
use optima_optimization::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization::nlopt::{Algorithm, NLOptOptimizer};
use optima_optimization::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{OwnedEmptyParryFilter, OwnedEmptyToProximityQry, OwnedParryDistanceAsProximityGroupQry, OwnedParryIntersectGroupQry, ParryDistanceGroupArgs, ParryIntersectGroupArgs, ParryPairSelector, ProximityLossFunction};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity::parry_ad::shape::Cuboid;
use optima_proximity::proxima::{OwnedParryProximaAsProximityQry, PairGroupQryArgsParryProxima, ProximaTermination};
use optima_proximity::shape_scene::OParryGenericShapeScene;
use optima_proximity::shapes::OParryShape;
use optima_robotics::robot::{ORobotDefault};
use optima_robotics::robotics_optimization::robotics_collision_state_resolver::{DifferentiableBlockCollisionStateResolver, DifferentiableFunctionCollisionStateResolver};
use optima_robotics::robotics_optimization::robotics_optimization_ik::{DifferentiableBlockIKObjectiveTrait, IKGoalUpdateMode};

fn main() {
    let robot = ORobotDefault::load_from_saved_robot("panda7");
    let robot = Arc::new(robot);
    let init_state = vec![1.3835341759012247, 0.644993809355163, -1.231244289043802, -1.599375351855726, -2.708679434572325, 2.9128314876962325, -2.173304133837488];
    let ik = robot.get_ik_differentiable_block(ForwardADMulti::<adfn<7>>::new(), OwnedEmptyParryFilter::new(()), OwnedEmptyToProximityQry::new(()), None, &init_state, vec![8], 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
    let o = SimpleOpEnOptimizer::new(robot.get_dof_lower_bounds(), robot.get_dof_upper_bounds(), 0.001);
    let pose_goal = Isometry3::from_constructors(&[0.6539468661968684,0.2720397734086678,0.3551723678894053], &QuatConstructor::new(0.2029080599152886, -0.6224496471954787, 0.3354943169507112, -0.6773686730440179));
    ik.update_ik_pose(0, pose_goal.clone(), IKGoalUpdateMode::Absolute);
    let res = o.optimize_unconstrained(&init_state, &ik);
    let solution = res.x_star().to_vec();
    println!("{:?}", res.solver_status().solve_time());

    let mut scene = OParryGenericShapeScene::new_empty();
    scene.add_shape(OParryShape::new_default(Cuboid::new(Vector3::new(0.029816824170463455 * 0.5, 1.4124879353697206 * 0.5, 0.7242890356101396 * 0.5)), Isometry3::identity()), Isometry3::from_constructors(&[1.190814282258407, -0.13550199346642616, 0.3621445178050698], &QuatConstructor {
        w: 0.9966893968483386,
        x: 0.0,
        y: 0.0,
        z: -0.0813034206543318,
    }));
    scene.add_shape(OParryShape::new_default(Cuboid::new(Vector3::new(0.5483168931687388 * 0.5, 1.4124879353697206 * 0.5, 0.029816824170463455 * 0.5)), Isometry3::identity()), Isometry3::from_constructors(&[0.9202803453390374, -0.09106954126692335, 0.13820469648641495], &QuatConstructor {
        w: 0.9966893968483386,
        x: 0.0,
        y: 0.0,
        z: -0.0813034206543318,
    }));
    scene.add_shape(OParryShape::new_default(Cuboid::new(Vector3::new(0.5483168931687388 * 0.5, 0.029816824170463455 * 0.5, 0.6159011632941881 * 0.5)), Isometry3::identity()), Isometry3::from_constructors(&[0.8058204345600789, -0.7879766159302007, 0.4312468660482773], &QuatConstructor {
        w: 0.9966893968483386,
        x: 0.0,
        y: 0.0,
        z: -0.0813034206543318,
    }));
    scene.add_shape(OParryShape::new_default(Cuboid::new(Vector3::new(0.5483168931687388 * 0.5, 0.029816824170463455 * 0.5, 0.6159011632941881 * 0.5)), Isometry3::identity()), Isometry3::from_constructors(&[1.034740256117996, 0.6058375333963542, 0.4312468660482773], &QuatConstructor {
        w: 0.9966893968483386,
        x: 0.0,
        y: 0.0,
        z: -0.0813034206543318,
    }));
    scene.add_shape(OParryShape::new_default(Cuboid::new(Vector3::new(0.5483168931687388 * 0.5, 0.029816824170463455 * 0.5, 0.6159011632941881 * 0.5)), Isometry3::identity()), Isometry3::from_constructors(&[0.927160824683477, -0.04917666628928477, 0.4312468660482773], &QuatConstructor {
        w: 0.9966893968483386,
        x: 0.0,
        y: 0.0,
        z: -0.0813034206543318,
    }));
    scene.add_shape(OParryShape::new_default(Cuboid::new(Vector3::new(0.5483168931687388 * 0.5, 1.4124879353697206 * 0.5, 0.029816824170463455 * 0.5)), Isometry3::identity()), Isometry3::from_constructors(&[0.927160824683477, -0.09106954126692335, 0.41456459296377046], &QuatConstructor {
        w: 0.9966893968483386,
        x: 0.0,
        y: 0.0,
        z: -0.0813034206543318,
    }));
    /*
    scene.add_shape(OParryShape::new_default(Cuboid::new(Vector3::new(0.3 * 0.5, 0.25 * 0.5, 0.8 * 0.5)), Isometry3::identity()), Isometry3::from_constructors(&[-0.05, 0.0, -0.4], &QuatConstructor {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }));
    */

    let control_points = vec![init_state.clone(), solution.clone()];
    let init_solution_path = InterpolatingSpline::new(control_points.clone(), InterpolatingSplineType::Linear).to_timed_interpolator(6.0);

    // let q = OwnedParryDistanceAsProximityGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryShapeRep::Full, ParryDisMode::ContactDis, false, false, -10000.0, true));
    let q = OwnedParryProximaAsProximityQry::new(PairGroupQryArgsParryProxima::new(ParryShapeRep::Full, ParryShapeRep::Full, false, false, ProximaTermination::MaxError(0.2), ProximityLossFunction::Hinge, 15.0, 0.2));
    let q2 = OwnedParryIntersectGroupQry::new(ParryIntersectGroupArgs::new(ParryShapeRep::Full, ParryShapeRep::Full, false, false));

    let mut max_proximity_val = f64::MIN;
    let mut max_proximity_state = init_state.clone();

    let ts = get_interpolation_range_num_steps(0.0, 1.0, 15);
    for t in &ts {
        let point = init_solution_path.interpolate_normalized(*t);
        let res = robot.parry_shape_scene_external_query(&point, &scene, &q, &ParryPairSelector::AllPairs, false);
        let res2 = robot.parry_shape_scene_external_query(&point, &scene, &q2, &ParryPairSelector::AllPairs, false);
        let proximity = res.get_proximity_objective_value(0.2, 15.0, ProximityLossFunction::Hinge);
        if proximity > max_proximity_val { max_proximity_val = proximity; max_proximity_state = point.clone(); }
        println!("{:?}", point);
        println!("{:?}", proximity);
        println!("{:?}", res2.intersect());
        println!("---");
    }

    let prox = OwnedParryDistanceAsProximityGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::OBB, ParryShapeRep::OBB, ParryDisMode::ContactDis, false, false, f64::MIN, false));
    // let prox = OwnedParryProximaAsProximityQry::new(PairGroupQryArgsParryProxima::new(ParryShapeRep::OBB, ParryShapeRep::OBB, false, false, ProximaTermination::MaxError(0.1), ProximityLossFunction::Hinge, 15.0, 0.1));
    let f1 = DifferentiableFunctionCollisionStateResolver::new(robot.clone(), Arc::new(scene.clone()), max_proximity_state.clone(), control_points[0].clone(), control_points[1].clone(), prox.clone(), prox, ParryPairSelector::HalfPairs, ParryPairSelector::AllPairs, 0.1, 15.0);
    let db2 = DifferentiableBlockCollisionStateResolver::new(ForwardADMulti::<adfn<7>>::new(), f1.to_other_ad_type::<f64>(), f1.to_other_ad_type::<adfn<7>>());

    // let o = SimpleOpEnOptimizer::new(robot.get_dof_lower_bounds(), robot.get_dof_upper_bounds(), 0.001);
    let o = NLOptOptimizer::new(Algorithm::Mma, false, 7, Some(10), None, None);
    let res = o.optimize_unconstrained(&max_proximity_state, &db2);
    println!("{:?}, {:?}", max_proximity_state, res.x_star());
    println!("{:?}", res);

    // let start = Instant::now();
    // let derivative = db2.derivative(&max_proximity_state);
    // let new_state = derivative.1.data.as_vec().ovec_scalar_mul(&-0.00001).ovec_add(&max_proximity_state);
    // println!("{:?}", new_state);

    let mut app = robot.bevy_get_motion_playback_app(&init_solution_path);
    app.optima_bevy_spawn_robot_in_pose(robot.clone(), max_proximity_state.clone(), 1);
    app.optima_bevy_spawn_robot_in_pose(robot.clone(), res.x_star().to_vec(), 2);
    app.optima_bevy_spawn_generic_shape_scene(scene.clone());

    app.run();
}