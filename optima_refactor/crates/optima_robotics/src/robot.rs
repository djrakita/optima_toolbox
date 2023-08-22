use serde::{Serialize, Deserialize};
use std::marker::PhantomData;
use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::vecs_and_mats::OLinalgTrait;
use serde_with::*;
use crate::chain::OChain;
use crate::robotics_components::*;
use crate::robotics_traits::{ChainableTrait, JointTrait};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ORobot<T: AD, P: O3DPose<T>, L: OLinalgTrait> {
    #[serde(deserialize_with = "Vec::<OChain<T, P, L>>::deserialize")]
    chains: Vec<OChain<T, P, L>>,
    #[serde(deserialize_with = "Vec::<OMacroJoint<T, P>>::deserialize")]
    macro_joints: Vec<OMacroJoint<T, P>>,
    macro_joint_sub_dof_idxs: Vec<Vec<usize>>,
    macro_joint_sub_dof_idxs_range: Vec<Option<(usize, usize)>>,
    chain_sub_dof_idxs: Vec<Vec<usize>>,
    chain_sub_dof_idxs_range: Vec<Option<(usize, usize)>>,
    chain_info: ChainInfo,
    _phantom_data: PhantomData<(T,P,L)>
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ORobot<T, P, L> {
    pub fn new_empty() -> Self {
        Self {
            chains: vec![ OChain::<T, P, L>::new_world_chain() ],
            macro_joints: vec![],
            macro_joint_sub_dof_idxs: vec![],
            macro_joint_sub_dof_idxs_range: vec![],
            chain_sub_dof_idxs: vec![],
            chain_sub_dof_idxs_range: vec![],
            chain_info: ChainInfo::new_empty(),
            _phantom_data: Default::default(),
        }
    }
    pub fn add_chain(&mut self, chain: OChain<T, P, L>, parent_chain_idx: usize, parent_link_idx_in_parent_chain: usize, origin: &P, axis: [T; 3], joint_type: OJointType) {
        assert!(parent_chain_idx <= self.chains.len());
        assert!(parent_link_idx_in_parent_chain <= self.chains[parent_chain_idx].links().len());

        let new_chain_idx = self.chains.len();
        let new_macro_joint_idx = self.macro_joints.len();
        let pose = OPose::from_o3d_pose(origin);
        let mut macro_joint = OMacroJoint::new(new_macro_joint_idx, pose, axis, joint_type, parent_chain_idx, parent_link_idx_in_parent_chain, new_chain_idx);
        self.chains.push(chain);
        self.macro_joints.push(macro_joint);

        self.setup();
    }
    pub fn chain_info(&self) -> &ChainInfo {
        &self.chain_info
    }
    pub fn chain_sub_dof_idxs(&self) -> &Vec<Vec<usize>> {
        &self.chain_sub_dof_idxs
    }
    pub fn chain_sub_dof_idxs_range(&self) -> &Vec<Option<(usize, usize)>> {
        &self.chain_sub_dof_idxs_range
    }
    fn setup(&mut self) {
        self.chain_info = self.compute_chain_info();
        self.set_all_sub_dof_idxs();
    }
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ORobot<T, P, L> {
    fn set_all_sub_dof_idxs(&mut self) {
        let mut count = 0;

        self.macro_joint_sub_dof_idxs = vec![ vec![]; self.macro_joints.len() ];
        self.macro_joint_sub_dof_idxs_range = vec![ None; self.macro_joints.len() ];
        self.chain_sub_dof_idxs = vec![ vec![]; self.chains.len() ];
        self.chain_sub_dof_idxs_range = vec![ None; self.chains.len() ];

        let kinematic_hierarchy = &self.chain_info.kinematic_hierarchy;
        for layer in kinematic_hierarchy {
            for chain_idx in layer {
                let parent_joint = self.chain_info.link_parent_joint(*chain_idx);
                match parent_joint {
                    None => { }
                    Some(parent_joint) => {
                        let num_dofs = self.macro_joints[*parent_joint].get_num_dofs();
                        if num_dofs > 0 {
                            let mut sub_dof_idxs = vec![];
                            for _ in 0..num_dofs {
                                sub_dof_idxs.push(count);
                                count += 1;
                            }
                            self.macro_joint_sub_dof_idxs_range[*parent_joint] = Some((*sub_dof_idxs.first().unwrap(), *sub_dof_idxs.last().unwrap() + 1));
                            self.macro_joint_sub_dof_idxs[*parent_joint] = sub_dof_idxs;
                        }
                    }
                }

                let num_dofs = self.chains[*chain_idx].num_dofs();
                if num_dofs > 0 {
                    let mut sub_dof_idxs = vec![];
                    for _ in 0..num_dofs {
                        sub_dof_idxs.push(count);
                        count += 1;
                    }
                    self.chain_sub_dof_idxs_range[*chain_idx] = Some((*sub_dof_idxs.first().unwrap(), *sub_dof_idxs.last().unwrap() + 1));
                    self.chain_sub_dof_idxs[*chain_idx] = sub_dof_idxs;
                }
            }
        }

    }
}

impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ChainableTrait<T, P> for ORobot<T, P, L> {
    type LinkType = OChain<T, P, L>;
    type JointType = OMacroJoint<T, P>;

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