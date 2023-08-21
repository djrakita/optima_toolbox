use serde::{Serialize, Deserialize};
use std::marker::PhantomData;
use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::vecs_and_mats::OLinalgTrait;
use serde_with::*;
use crate::chain::OChain;
use crate::robotics_components::*;
use crate::robotics_traits::{ChainableTrait};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ORobot<T: AD, P: O3DPose<T>, L: OLinalgTrait> {
    #[serde(deserialize_with = "Vec::<OChain<T, P, L>>::deserialize")]
    chains: Vec<OChain<T, P, L>>,
    #[serde(deserialize_with = "Vec::<OChainJoint<T, P>>::deserialize")]
    macro_joints: Vec<OChainJoint<T, P>>,
    _phantom_data: PhantomData<(T,P,L)>
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ORobot<T, P, L> {
    pub fn new_empty() -> Self {
        Self {
            chains: vec![ OChain::<T, P, L>::new_world_chain() ],
            macro_joints: vec![],
            _phantom_data: Default::default(),
        }
    }
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ChainableTrait for ORobot<T, P, L> {
    type LinkType = OChain<T, P, L>;
    type JointType = OChainJoint<T, P>;

    fn links(&self) -> &Vec<Self::LinkType> {
        &self.chains
    }

    fn joints(&self) -> &Vec<Self::JointType> {
        &self.macro_joints
    }
}

impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> OChain<T, P, L> {
    pub (crate) fn new_world_chain() -> Self {
        Self::from_manual("world", vec![OLink::new_manual("world_link", vec![], vec![], OInertial::new_zeros())], vec![])
    }
}