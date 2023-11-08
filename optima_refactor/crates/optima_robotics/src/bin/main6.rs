use optima_proximity2::pair_group_queries::{OPairGroupQryTrait, ParryDistanceGroupArgs, ParryDistanceGroupQry, ParryIntersectGroupArgs, ParryIntersectGroupQry, ParryPairSelector};
use optima_proximity2::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity2::shape_scene::ShapeSceneTrait;
use optima_robotics::robot::ORobotDefault;

fn main() {
    // let mut r = ORobotDefault::from_urdf("ur5");
    // r.preprocess(100);
    // r.save_robot(None);

    let r = ORobotDefault::load_from_saved_robot("ur5");

    let s = r.parry_shape_scene().get_shapes();
    let p = r.parry_shape_scene().get_poses(&(&r, &[0.0, 0.0, 2.87, 0.0, 0.0, 0.0]));
    let skips = r.parry_shape_scene().get_pair_skips();

    let res = ParryDistanceGroupQry::query(&s, &s, &p, &p, &ParryPairSelector::HalfPairs, skips, &ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, -1000000.0));
    println!("{:?}", res.min_dis());
}