use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_bevy::optima_bevy_utils::robotics::BevyRoboticsTrait;
use optima_proximity::pair_group_queries::{OPairGroupQryTrait, OwnedPairGroupQry, ParryDistanceBoundsGroupOutput};
use optima_robotics::robot::{ORobotDefault};

fn test<T: AD, P: O3DPose<T>, Q: OPairGroupQryTrait<T, P, Output=ParryDistanceBoundsGroupOutput<T>>>(q: OwnedPairGroupQry<T, P, Q>) {
    todo!()
}

fn main() {
    let mut r = ORobotDefault::load_from_saved_robot("xarm7");
    // r.parry_shape_scene_compute_average_distances(SaveRobot::Save(None), Some(2000));
    r.bevy_self_collision_visualization();
}