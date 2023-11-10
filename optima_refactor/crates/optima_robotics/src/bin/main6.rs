use optima_proximity2::pair_group_queries::{OPairGroupQryTrait, ParryDistanceGroupArgs, ParryDistanceGroupQry, ParryDistanceWrtAverageGroupArgs, ParryDistanceWrtAverageGroupQry, ParryIntersectGroupArgs, ParryIntersectGroupQry, ParryPairSelector};
use optima_proximity2::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity2::shape_scene::ShapeSceneTrait;
use optima_robotics::robot::{ORobotDefault};


fn main() {
    let mut r = ORobotDefault::load_from_saved_robot("ur5");
    let s = r.parry_shape_scene().get_shapes();
    let state = vec![0.,0.,2.85,0.,0.,0.];
    let p = r.parry_shape_scene().get_poses(&(&r, &state));
    let av = r.parry_shape_scene().get_pair_average_distances();
    let res = ParryDistanceWrtAverageGroupQry::query(s, s, p.as_ref(), p.as_ref(), &ParryPairSelector::HalfPairsSubcomponents, r.parry_shape_scene().get_pair_skips(), &ParryDistanceWrtAverageGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, av, -100000000.0));
    println!("{:?}", res.outputs()[0].data().raw_distance());
    println!("{:?}", res.outputs()[0].data().distance_wrt_average());
    println!("{:?}", res.min_dis_wrt_average());
    println!("{:?}", res.outputs()[1].data().raw_distance());
    println!("{:?}", res.outputs()[1].data().distance_wrt_average());
}