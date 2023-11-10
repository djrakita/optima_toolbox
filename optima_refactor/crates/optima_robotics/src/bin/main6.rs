use nalgebra::Isometry3;
use optima_proximity2::pair_group_queries::{OPairGroupQryTrait, ParryDistanceGroupArgs, ParryDistanceGroupQry, ParryIntersectGroupArgs, ParryIntersectGroupQry, ParryPairSelector};
use optima_proximity2::pair_queries::{ParryDisMode, ParryShapeRep};
use optima_proximity2::shape_queries::IntersectOutputTrait;
use optima_proximity2::shape_scene::ShapeSceneTrait;
use optima_robotics::robot::{ORobotDefault, SaveRobot};
use optima_robotics::robot_set::ORobotSetDefault;
use optima_robotics::robotics_components::{OJointLimit, OJointType};
use optima_robotics::robotics_traits::AsRobotTrait;

fn main() {
    // let mut r = ORobotDefault::from_urdf("b1");
    // r.preprocess(SaveRobot::Save(None), None, None);

    // let mut r = ORobotDefault::from_urdf("ur5");
    // r.preprocess(SaveRobot::Save(None), None, None);

    let mut r = ORobotDefault::load_from_saved_robot("ur5");
    // r.add_non_collision_state(vec![0.,0.,2.,0.,0.,0.], SaveRobot::Save(None));
    let s = r.parry_shape_scene().get_shapes();
    let state = vec![0.,0.,2.849,0.,0.,0.];
    let p = r.parry_shape_scene().get_poses(&(&r, &state));
    // let res = ParryDistanceGroupQry::query(s, s, p.as_ref(), p.as_ref(), &ParryPairSelector::HalfPairsSubcomponents, r.parry_shape_scene().get_pair_skips(), &ParryDistanceGroupArgs::new(ParryShapeRep::Full, ParryDisMode::ContactDis, -100000000.0));
    let res = ParryIntersectGroupQry::query(s, s, p.as_ref(), p.as_ref(), &ParryPairSelector::HalfPairsSubcomponents, r.parry_shape_scene().get_pair_skips(), &ParryIntersectGroupArgs::new(ParryShapeRep::Full, false));
    println!("{:?}", res.intersect());

    res.outputs().iter().for_each(|x| {
       if x.data().intersect() {
           let ids = x.pair_ids();
           println!("({:?}, {:?})", r.parry_shape_scene().shape_id_to_shape_str(ids.0), r.parry_shape_scene().shape_id_to_shape_str(ids.1))
       }
    });

    let fk_res = r.forward_kinematics(&state, None);

    fk_res.link_poses().iter().enumerate().for_each(|(i, x)| {
        if let Some(x) = x {
            // println!("{:?}: {:?}", i, x);
        }
    });
}