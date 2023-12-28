use std::borrow::Cow;
use ad_trait::differentiable_function::FiniteDifferencing;
use optima_interpolation::splines::{SplineConstructorLinear, SplineConstructorTrait};
use optima_robotics::robotics_optimization::path_optimization::{DifferentiableBlockPathOpt, DifferentiableFunctionPathOpt};
use nalgebra::Isometry3;
use optima_3d_spatial::optima_3d_pose::O3DPoseCategoryIsometry3;
use optima_bevy::{App, OptimaBevyTrait};
use optima_bevy::optima_bevy_utils::viewport_visuals::BevyDrawShape;
use optima_linalg::{OVec, VecOfOVecTrait};
use optima_optimization::{DiffBlockOptimizerTrait, OptimizerOutputTrait};
use optima_optimization::open::SimpleOpEnOptimizer;
use optima_proximity::pair_group_queries::{OwnedParryDistanceAsProximityGroupQry, ParryDistanceGroupArgs};
use optima_proximity::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity::shape_scene::OParryGenericShapeScene;
use optima_proximity::shapes::OParryShape;
use optima_proximity::parry_ad::shape::Ball;

fn main() {
    let mut env = OParryGenericShapeScene::new_empty();
    env.add_shape(OParryShape::new_default(Ball::new(0.5), Isometry3::identity()), Isometry3::identity());
    let start_point = vec![-2.0, 0.1,0.];
    let end_point = vec![2.0, 0., 0.];
    let spline_constructor = SplineConstructorLinear;
    let q = OwnedParryDistanceAsProximityGroupQry::new(ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryShapeRep::Full, ParryDisMode::ContactDis, false, false, -1000.0, false));
    let f: DifferentiableFunctionPathOpt<_, O3DPoseCategoryIsometry3, _, _> = DifferentiableFunctionPathOpt::new(spline_constructor.clone(), Cow::Owned(env.clone()), q, start_point.clone(), end_point.clone(), 20, 20, 4.0, 5.0, 0.0, 0., 0.3);
    let f2 = f.to_other_ad_type::<f64>();
    let db = DifferentiableBlockPathOpt::new(FiniteDifferencing::new(), f, f2);

    let initial_condition = spline_constructor.get_default_initial_condition(start_point.clone(), end_point.clone(), 10);
    let initial_condition = initial_condition.concatenate_all();
    let o = SimpleOpEnOptimizer::new(vec![-1000.0; initial_condition.len()], vec![1000.0; initial_condition.len()], 0.001);
    let res = o.optimize_unconstrained(&initial_condition, &db);
    println!("{:?}", res.x_star());
    println!("{:?}", res.solver_status().has_converged());
    println!("{:?}", res.f_star());

    let spline_answer = spline_constructor.construct(res.x_star().to_vec().ovec_split_into_sub_vecs_owned(3));

    let mut app = App::new();
    app.optima_bevy_starter_scene()
        .optima_bevy_draw_3d_curve(spline_answer, 1000, 5.0, 6, 2)
        .optima_bevy_draw_shape(BevyDrawShape::Sphere { radius: 0.1 }, Isometry3::translation(-2.0, 0.1, 0.))
        .optima_bevy_draw_shape(BevyDrawShape::Sphere { radius: 0.1 }, Isometry3::translation(2.0, 0., 0.))
        .optima_bevy_spawn_generic_shape_scene(env.clone());
    app.run();
}