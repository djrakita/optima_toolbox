use ad_trait::AD;
use as_any::AsAny;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::OVec;

pub trait OProximitySceneTrait<T: AD, ShapeType, PoseType: O3DPose<T>> {
    type Input<V: OVec<T>> : AsAny;

    fn get_shapes(&self) -> &Vec<ShapeType>;
    fn get_proximity_scene_poses<V: OVec<T>>(&self, input: &Self::Input<V>) -> Vec<PoseType>;
}

