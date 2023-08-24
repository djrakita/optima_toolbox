use serde::{Serialize, Deserialize};
use std::marker::PhantomData;
use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::vecs_and_mats::{OLinalgTrait, OVec};
use serde_with::*;
use crate::chain::OChain;
use crate::robotics_components::*;
use crate::robotics_traits::{ChainableTrait, JointTrait};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ORobot<T: AD, P: O3DPose<T>, L: OLinalgTrait> {
    #[serde(deserialize_with = "Vec::<OChainWrapper<T, P, L>>::deserialize")]
    chain_wrappers: Vec<OChainWrapper<T, P, L>>,
    #[serde(deserialize_with = "Vec::<OMacroJoint<T, P>>::deserialize")]
    macro_joints: Vec<OMacroJoint<T, P>>,
    kinematic_hierarchy: Vec<Vec<usize>>,
    num_dofs: usize,
    base_chain_idx: usize,
    dof_to_joint_and_sub_dof_idxs: Vec<RobotJointIdx>,
    _phantom_data: PhantomData<(T,P,L)>
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ORobot<T, P, L> {
    pub fn new_empty() -> Self {
        Self {
            chain_wrappers: vec![ OChainWrapper::<T, P, L>::new_world_chain() ],
            macro_joints: vec![],
            kinematic_hierarchy: vec![],
            num_dofs: usize::default(),
            base_chain_idx: usize::default(),
            dof_to_joint_and_sub_dof_idxs: vec![],
            _phantom_data: Default::default(),
        }
    }
    pub fn add_chain(&mut self, chain: OChain<T, P, L>, parent_chain_idx: usize, parent_link_idx_in_parent_chain: usize, origin: &P, axis: [T; 3], joint_type: OJointType, limit: OJointLimit<T>) {
        assert!(parent_chain_idx <= self.chain_wrappers.len());
        assert!(parent_link_idx_in_parent_chain <= self.chain_wrappers[parent_chain_idx].chain.links().len());

        let new_chain_idx = self.chain_wrappers.len();
        let new_macro_joint_idx = self.macro_joints.len();
        let pose = OPose::from_o3d_pose(origin);
        let macro_joint = OMacroJoint::new(new_macro_joint_idx, pose, axis, joint_type, limit, parent_chain_idx, parent_link_idx_in_parent_chain, new_chain_idx);

        let chain_wrapper = OChainWrapper {
            chain,
            chain_idx: new_chain_idx,
            parent_joint_idx: None,
            children_joint_idxs: vec![],
            parent_link_idx: None,
            children_link_idxs: vec![],
            chain_connection_paths: vec![],
            dof_idxs: vec![],
            dof_idxs_range: None,
        };

        self.chain_wrappers.push(chain_wrapper);
        self.macro_joints.push(macro_joint);

        self.setup();
    }
    pub fn chain_wrappers(&self) -> &Vec<OChainWrapper<T, P, L>> {
        &self.chain_wrappers
    }
    pub fn macro_joints(&self) -> &Vec<OMacroJoint<T, P>> {
        &self.macro_joints
    }
    pub fn kinematic_hierarchy(&self) -> &Vec<Vec<usize>> {
        &self.kinematic_hierarchy
    }
    pub fn num_dofs(&self) -> usize {
        self.num_dofs
    }
    pub fn base_chain_idx(&self) -> usize {
        self.base_chain_idx
    }
    #[inline]
    pub fn get_macro_joint_transform<V: OVec<T>>(&self, state: &V, macro_joint_idx: usize) -> P {
        self.macro_joints[macro_joint_idx].get_joint_transform(state, &self.macro_joints)
    }
    #[inline(always)]
    pub fn get_macro_joint_fixed_offset_transform(&self, macro_joint_idx: usize) -> &P {
        self.macro_joints[macro_joint_idx].get_joint_fixed_offset_transform()
    }
    #[inline]
    pub fn get_macro_joint_variable_transform<V: OVec<T>>(&self, state: &V, macro_joint_idx: usize) -> P {
        self.macro_joints[macro_joint_idx].get_joint_variable_transform(state, &self.macro_joints)
    }
    #[inline]
    pub fn dof_to_joint_and_sub_dof_idxs(&self) -> &Vec<RobotJointIdx> {
        &self.dof_to_joint_and_sub_dof_idxs
    }
    fn setup(&mut self) {
        self.set_chain_info();
        self.set_num_dofs();
        self.set_all_sub_dof_idxs();
        self.set_dof_to_joint_and_sub_dof_idxs();
    }
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ORobot<T, P, L> {
    fn set_chain_info(&mut self) {
        let chain_info = self.compute_chain_info();

        self.base_chain_idx = chain_info.base_link_idx;
        self.kinematic_hierarchy = chain_info.kinematic_hierarchy.clone();

        self.chain_wrappers.iter_mut().enumerate().for_each(|(i, x)| {
            x.parent_joint_idx = chain_info.link_parent_joint[i].clone();
            x.children_joint_idxs = chain_info.link_children_joints[i].clone();
            x.parent_link_idx = chain_info.link_parent_link[i].clone();
            x.children_link_idxs = chain_info.link_children_links[i].clone();
            x.chain_connection_paths = chain_info.link_connection_paths[i].clone();
        });
    }
    fn set_num_dofs(&mut self) {
        let mut num_dofs = 0;
        self.chain_wrappers.iter().for_each(|x| num_dofs += x.chain.num_dofs() );
        self.macro_joints.iter().for_each(|x| num_dofs += x.get_num_dofs() );

        self.num_dofs = num_dofs;
    }
    fn set_all_sub_dof_idxs(&mut self) {
        let mut count = 0;

        let mut macro_joint_sub_dof_idxs = vec![ vec![]; self.macro_joints.len() ];
        let mut macro_joint_sub_dof_idxs_range = vec![ None; self.macro_joints.len() ];
        let mut chain_sub_dof_idxs = vec![ vec![]; self.chain_wrappers.len() ];
        let mut chain_sub_dof_idxs_range = vec![ None; self.chain_wrappers.len() ];

        let kinematic_hierarchy = &self.kinematic_hierarchy;
        for layer in kinematic_hierarchy {
            for chain_idx in layer {
                // let parent_joint = self.chain_info.link_parent_joint(*chain_idx);
                let parent_joint = self.chain_wrappers[*chain_idx].parent_joint_idx;
                match parent_joint {
                    None => { }
                    Some(parent_joint) => {
                        let num_dofs = self.macro_joints[parent_joint].get_num_dofs();
                        if num_dofs > 0 {
                            let mut sub_dof_idxs = vec![];
                            for _ in 0..num_dofs {
                                sub_dof_idxs.push(count);
                                count += 1;
                            }
                            macro_joint_sub_dof_idxs_range[parent_joint] = Some((*sub_dof_idxs.first().unwrap(), *sub_dof_idxs.last().unwrap() + 1));
                            macro_joint_sub_dof_idxs[parent_joint] = sub_dof_idxs;
                        }
                    }
                }

                let num_dofs = self.chain_wrappers[*chain_idx].chain.num_dofs();
                if num_dofs > 0 {
                    let mut sub_dof_idxs = vec![];
                    for _ in 0..num_dofs {
                        sub_dof_idxs.push(count);
                        count += 1;
                    }
                    chain_sub_dof_idxs_range[*chain_idx] = Some((*sub_dof_idxs.first().unwrap(), *sub_dof_idxs.last().unwrap() + 1));
                    chain_sub_dof_idxs[*chain_idx] = sub_dof_idxs;
                }
            }
        }

        self.chain_wrappers.iter_mut().enumerate().for_each(|(i, x)| {
            x.dof_idxs = chain_sub_dof_idxs[i].clone();
            x.dof_idxs_range = chain_sub_dof_idxs_range[i].clone();
        });

        self.macro_joints.iter_mut().enumerate().for_each(|(i, x)| {
            x.dof_idxs = macro_joint_sub_dof_idxs[i].clone();
            x.dof_idxs_range = macro_joint_sub_dof_idxs_range[i].clone();
        });
    }
    fn set_dof_to_joint_and_sub_dof_idxs(&mut self) {
        let mut dof_to_joint_and_sub_dof_idxs = vec![ RobotJointIdx::default(); self.num_dofs ];

        self.macro_joints.iter().enumerate().for_each(|(macro_joint_idx, macro_joint)| {
            macro_joint.dof_idxs.iter().enumerate().for_each(|(i, dof_idx)| {
                dof_to_joint_and_sub_dof_idxs[*dof_idx] = RobotJointIdx::MacroJoint { macro_joint_idx, sub_dof_idx: i };
            });
        });

        self.chain_wrappers.iter().enumerate().for_each(|(chain_idx, chain_wrapper)| {
            chain_wrapper.dof_idxs.iter().enumerate().for_each(|(i, dof_idx)| {
                let (joint_idx, sub_dof_idx) = chain_wrapper.chain.dof_to_joint_and_sub_dof_idxs()[i].clone();

                dof_to_joint_and_sub_dof_idxs[*dof_idx] = RobotJointIdx::ChainJoint {
                    chain_idx,
                    joint_idx,
                    sub_dof_idx,
                };
            });
        });

        self.dof_to_joint_and_sub_dof_idxs = dof_to_joint_and_sub_dof_idxs;
    }
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ChainableTrait<T, P> for ORobot<T, P, L> {
    type LinkType = OChainWrapper<T, P, L>;
    type JointType = OMacroJoint<T, P>;

    fn links(&self) -> &Vec<Self::LinkType> {
        &self.chain_wrappers
    }

    fn joints(&self) -> &Vec<Self::JointType> {
        &self.macro_joints
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OChainWrapper<T: AD, P: O3DPose<T>, L: OLinalgTrait> {
    #[serde(deserialize_with = "OChain::<T, P, L>::deserialize")]
    chain: OChain<T, P, L>,
    chain_idx: usize,
    pub (crate) parent_joint_idx: Option<usize>,
    pub (crate) children_joint_idxs: Vec<usize>,
    pub (crate) parent_link_idx: Option<usize>,
    pub (crate) children_link_idxs: Vec<usize>,
    pub (crate) chain_connection_paths: Vec<Option<Vec<usize>>>,
    dof_idxs: Vec<usize>,
    dof_idxs_range: Option<(usize, usize)>
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> OChainWrapper<T, P, L> {
    fn new_world_chain() -> Self {
        Self {
            chain: OChain::new_world_chain(),
            chain_idx: 0,
            parent_joint_idx: None,
            children_joint_idxs: vec![],
            parent_link_idx: None,
            children_link_idxs: vec![],
            chain_connection_paths: vec![],
            dof_idxs: vec![ ],
            dof_idxs_range: None,
        }
    }
    #[inline(always)]
    pub fn chain(&self) -> &OChain<T, P, L> {
        &self.chain
    }
    #[inline(always)]
    pub fn chain_idx(&self) -> usize {
        self.chain_idx
    }
    #[inline(always)]
    pub fn sub_dof_idxs(&self) -> &Vec<usize> {
        &self.dof_idxs
    }
    #[inline(always)]
    pub fn sub_dof_idxs_range(&self) -> &Option<(usize, usize)> {
        &self.dof_idxs_range
    }
    #[inline(always)]
    pub fn parent_joint_idx(&self) -> &Option<usize> {
        &self.parent_joint_idx
    }
    #[inline(always)]
    pub fn children_joint_idxs(&self) -> &Vec<usize> {
        &self.children_joint_idxs
    }
    #[inline(always)]
    pub fn parent_link_idx(&self) -> &Option<usize> {
        &self.parent_link_idx
    }
    #[inline(always)]
    pub fn children_link_idxs(&self) -> &Vec<usize> {
        &self.children_link_idxs
    }
    #[inline(always)]
    pub fn chain_connection_paths(&self) -> &Vec<Option<Vec<usize>>> {
        &self.chain_connection_paths
    }
}

impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> OChain<T, P, L> {
    pub (crate) fn new_world_chain() -> Self {
        Self::from_manual("world", vec![OLink::new_manual("world_link", vec![], vec![], OInertial::new_zeros())], vec![])
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum RobotJointIdx {
    ChainJoint { chain_idx: usize, joint_idx: usize, sub_dof_idx: usize },
    MacroJoint { macro_joint_idx: usize, sub_dof_idx: usize }
}
impl Default for RobotJointIdx {
    fn default() -> Self {
        Self::ChainJoint {
            chain_idx: 0,
            joint_idx: 0,
            sub_dof_idx: 0,
        }
    }
}