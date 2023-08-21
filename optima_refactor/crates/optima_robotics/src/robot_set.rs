use serde::{Serialize, Deserialize};
use std::marker::PhantomData;
use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::vecs_and_mats::OLinalgTrait;
use crate::robot::ORobot;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ORobotSet<T: AD, P: O3DPose<T>, L: OLinalgTrait> {
    #[serde(deserialize_with = "Vec::<ORobot<T, P, L>>::deserialize")]
    robots: Vec<ORobot<T, P, L>>,
    _phantom_data: PhantomData<(T,P,L)>
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ORobotSet<T, P, L> {
    pub fn new_empty() -> Self {
        Self {
            robots: vec![],
            _phantom_data: Default::default(),
        }
    }
}