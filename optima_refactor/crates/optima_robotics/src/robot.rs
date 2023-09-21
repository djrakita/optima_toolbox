use std::collections::HashMap;
use std::fmt::Debug;
use serde::{Serialize, Deserialize};
use std::marker::PhantomData;
use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryIsometry3, O3DPoseCategoryTrait};
use optima_linalg::{OLinalgCategoryNalgebra, OLinalgCategoryTrait, OVec};
use serde_with::*;
use optima_file::traits::{FromJsonString, ToJsonString};
use crate::chain::{ChainFKResult, OChain};
use crate::robotics_components::*;
use crate::robotics_functions::compute_chain_info;
use crate::robotics_traits::{AsChainTrait, JointTrait};

pub type ORobotDefault = ORobot<f64, O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra>;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ORobot<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> {
    #[serde(deserialize_with = "Vec::<OChainWrapper<T, C, L>>::deserialize")]
    chain_wrappers: Vec<OChainWrapper<T, C, L>>,
    #[serde(deserialize_with = "Vec::<OMacroJoint<T, C>>::deserialize")]
    macro_joints: Vec<OMacroJoint<T, C>>,
    kinematic_hierarchy_of_chains: Vec<Vec<usize>>,
    num_dofs: usize,
    base_chain_idx: usize,
    dof_to_joint_and_sub_dof_idxs: Vec<RobotJointIdx>,
    #[serde(deserialize_with = "OChain::<T, C, L>::deserialize")]
    robot_as_single_chain: OChain<T, C, L>,
    _phantom_data: PhantomData<(T, C, L)>,
}
impl<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> ORobot<T, C, L> {
    pub fn new_empty() -> Self {
        Self {
            chain_wrappers: vec![OChainWrapper::<T, C, L>::new_world_chain()],
            macro_joints: vec![],
            kinematic_hierarchy_of_chains: vec![],
            num_dofs: usize::default(),
            base_chain_idx: usize::default(),
            dof_to_joint_and_sub_dof_idxs: vec![],
            robot_as_single_chain: OChain::new_empty(),
            _phantom_data: Default::default(),
        }
    }
    pub fn new_from_single_chain(chain: OChain<T, C, L>) -> Self {
        let mut out = Self::new_empty();
        out.add_chain(chain, 0, 0, &C::P::<T>::identity(), [T::zero(); 3], OJointType::Fixed, OJointLimit::default());
        out
    }
    pub fn new_from_single_chain_name(chain_name: &str) -> Self {
        Self::new_from_single_chain(OChain::from_urdf(chain_name))
    }
    pub fn to_new_generic_types<T2: AD, C2: O3DPoseCategoryTrait, L2: OLinalgCategoryTrait>(&self) -> ORobot<T2, C2, L2> {
        let json_str = self.to_json_string();
        ORobot::<T2, C2, L2>::from_json_string(&json_str)
    }
    pub fn to_new_ad_type<T2: AD>(&self) -> ORobot<T2, C, L> {
        self.to_new_generic_types::<T2, C, L>()
    }
    pub fn add_chain(&mut self, chain: OChain<T, C, L>, parent_chain_idx: usize, parent_link_idx_in_parent_chain: usize, origin: &C::P<T>, axis: [T; 3], joint_type: OJointType, limit: OJointLimit<T>) {
        assert!(parent_chain_idx <= self.chain_wrappers.len());
        assert!(parent_link_idx_in_parent_chain <= self.chain_wrappers[parent_chain_idx].chain.joints().len());
        assert!(limit.upper().len() >= joint_type.num_dofs());
        assert!(limit.lower().len() >= joint_type.num_dofs());
        assert!(limit.velocity().len() >= joint_type.num_dofs());
        assert!(limit.effort().len() >= joint_type.num_dofs());

        let new_chain_idx = self.chain_wrappers.len();
        let new_macro_joint_idx = self.macro_joints.len();
        let pose = OPose::from_o3d_pose(origin);
        let macro_joint = OMacroJoint::new(new_macro_joint_idx, pose, axis, joint_type, limit, parent_chain_idx, parent_link_idx_in_parent_chain, new_chain_idx);

        let chain_wrapper = OChainWrapper {
            chain,
            chain_idx: new_chain_idx,
            parent_macro_joint_idx: None,
            children_macro_joint_idxs: vec![],
            parent_chain_idx: None,
            parent_link_idx_in_parent_chain: None,
            children_chain_idxs: vec![],
            chain_connection_paths: vec![],
            dof_idxs: vec![],
            dof_idxs_range: None,
        };

        self.chain_wrappers.push(chain_wrapper);
        self.macro_joints.push(macro_joint);

        self.setup();
    }
    #[inline(always)]
    pub fn chain_wrappers(&self) -> &Vec<OChainWrapper<T, C, L>> {
        &self.chain_wrappers
    }
    #[inline(always)]
    pub fn macro_joints(&self) -> &Vec<OMacroJoint<T, C>> {
        &self.macro_joints
    }
    #[inline(always)]
    pub fn kinematic_hierarchy_of_chains(&self) -> &Vec<Vec<usize>> {
        &self.kinematic_hierarchy_of_chains
    }
    #[inline(always)]
    pub fn num_dofs(&self) -> usize {
        self.num_dofs
    }
    #[inline(always)]
    pub fn base_chain_idx(&self) -> usize {
        self.base_chain_idx
    }
    #[inline]
    pub fn get_macro_joint_transform<V: OVec<T>>(&self, state: &V, macro_joint_idx: usize) -> C::P<T> {
        self.macro_joints[macro_joint_idx].get_joint_transform(state, &self.macro_joints)
    }
    #[inline(always)]
    pub fn get_macro_joint_fixed_offset_transform(&self, macro_joint_idx: usize) -> &C::P<T> {
        self.macro_joints[macro_joint_idx].get_joint_fixed_offset_transform()
    }
    #[inline]
    pub fn get_macro_joint_variable_transform<V: OVec<T>>(&self, state: &V, macro_joint_idx: usize) -> C::P<T> {
        self.macro_joints[macro_joint_idx].get_joint_variable_transform(state, &self.macro_joints)
    }
    #[inline]
    pub fn dof_to_joint_and_sub_dof_idxs(&self) -> &Vec<RobotJointIdx> {
        &self.dof_to_joint_and_sub_dof_idxs
    }
    pub fn forward_kinematics<V: OVec<T>>(&self, state: &V, base_offset: Option<&C::P<T>>) -> RobotFKResult<T, C::P<T>> {
        assert_eq!(self.num_dofs(), state.len(), "The state length {} must match the number of dofs {}", state.len(), self.num_dofs());

        let mut out = vec![None; self.chain_wrappers.len()];

        let base_pose = match base_offset {
            None => { C::P::<T>::identity() }
            Some(base_offset) => { base_offset.to_owned() }
        };

        self.chain_wrappers().iter().enumerate().for_each(|(chain_idx, chain_wrapper)| {
            let dof_idxs_range = &chain_wrapper.dof_idxs_range;
            if chain_idx == 0 {
                let fk_res = match dof_idxs_range {
                    None => { chain_wrapper.chain.forward_kinematics(&[], Some(&base_pose)) }
                    Some(r) => { chain_wrapper.chain.forward_kinematics(&state.subslice(r.0, r.1), Some(&base_pose)) }
                };
                out[chain_idx] = Some(fk_res);
            } else {
                let parent_chain_idx = chain_wrapper.parent_chain_idx.unwrap();
                let parent_link_idx_in_parent_chain = chain_wrapper.parent_link_idx_in_parent_chain.unwrap();
                let parent_macro_joint_idx = chain_wrapper.parent_macro_joint_idx.unwrap();

                let pose = out[parent_chain_idx].as_ref().unwrap().get_link_pose(parent_link_idx_in_parent_chain).as_ref().expect("pose must not be None");
                let transform = self.get_macro_joint_transform(state, parent_macro_joint_idx);

                let base_offset = pose.mul(&transform);

                let fk_res = match dof_idxs_range {
                    None => { chain_wrapper.chain.forward_kinematics(&[], Some(&base_pose)) }
                    Some(r) => { chain_wrapper.chain.forward_kinematics(&state.subslice(r.0, r.1), Some(&base_offset)) }
                };

                out[chain_idx] = Some(fk_res);
            }
        });

        RobotFKResult { chain_fk_results: out, _phantom_data: Default::default() }
    }
    pub fn forward_kinematics_floating_chain<V: OVec<T>>(&self, state: &V, start_chain_idx: usize, end_chain_idx: usize, base_offset: Option<&C::P<T>>) -> RobotFKResult<T, C::P<T>> {
        let mut out = vec![None; self.chain_wrappers.len()];

        let chain_path = &self.chain_wrappers[start_chain_idx].chain_connection_paths[end_chain_idx];
        match chain_path {
            None => {}
            Some(chain_path) => {
                let base_pose = match base_offset {
                    None => { C::P::<T>::identity() }
                    Some(base_offset) => { base_offset.to_owned() }
                };
                chain_path.iter().enumerate().for_each(|(i, chain_idx)| {
                    let chain_wrapper = &self.chain_wrappers[*chain_idx];
                    let dof_idxs_range = &chain_wrapper.dof_idxs_range;
                    if i == 0 {
                        let fk_res = match dof_idxs_range {
                            None => { chain_wrapper.chain.forward_kinematics(&[], Some(&base_pose)) }
                            Some(r) => { chain_wrapper.chain.forward_kinematics(&state.subslice(r.0, r.1), Some(&base_pose)) }
                        };
                        out[*chain_idx] = Some(fk_res);
                    } else {
                        let parent_chain_idx = chain_wrapper.parent_chain_idx.unwrap();
                        let parent_link_idx_in_parent_chain = chain_wrapper.parent_link_idx_in_parent_chain.unwrap();
                        let parent_macro_joint_idx = chain_wrapper.parent_macro_joint_idx.unwrap();

                        let pose = out[parent_chain_idx].as_ref().unwrap().get_link_pose(parent_link_idx_in_parent_chain).as_ref().expect("pose must not be None");
                        let transform = self.get_macro_joint_transform(state, parent_macro_joint_idx);

                        let base_offset = pose.mul(&transform);

                        let fk_res = match dof_idxs_range {
                            None => { chain_wrapper.chain.forward_kinematics(&[], Some(&base_pose)) }
                            Some(r) => { chain_wrapper.chain.forward_kinematics(&state.subslice(r.0, r.1), Some(&base_offset)) }
                        };

                        out[*chain_idx] = Some(fk_res);
                    }
                });
            }
        }

        RobotFKResult { chain_fk_results: out, _phantom_data: Default::default() }
    }
    pub (crate) fn get_robot_as_single_chain(&self) -> OChain<T, C, L> {
        let mut links = vec![];
        let mut joints = vec![];

        let mut chain_and_link_idx_to_new_link_name_mapping = HashMap::new();

        self.chain_wrappers.iter().enumerate().for_each(|(chain_idx, chain_wrapper)| {
            chain_wrapper.chain.links().iter().enumerate().for_each(|(link_idx_in_chain, link)| {
                let link_name = format!("chain_{}_{}", chain_idx, link.name());
                chain_and_link_idx_to_new_link_name_mapping.insert((chain_idx as usize, link_idx_in_chain as usize), link_name.clone());
                let mut new_link = link.clone();
                new_link.name = link_name.clone();
                new_link.sub_chain_idx = chain_idx;
                new_link.link_idx_in_sub_chain = link_idx_in_chain;
                links.push(new_link);
                // links.push(new_link.to_new_auxiliary_info_type(RobotLinkAuxInfo { chain_idx, link_idx_in_chain }));
                // links.push(OLink::new_manual(&link_name, link.collision().clone(), link.visual().clone(), link.inertial().clone(), RobotLinkAuxInfo { chain_idx, link_idx_in_chain }))
            });
        });

        self.kinematic_hierarchy_of_chains.iter().for_each(|chain_idx_layer| {
            chain_idx_layer.iter().for_each(|chain_idx| {
                let chain_wrapper = self.chain_wrappers.get(*chain_idx).expect("error");
                let chain = chain_wrapper.chain();
                let chain_base_link_idx = chain.base_link_idx();

                let parent_chain_idx = chain_wrapper.parent_chain_idx;
                let parent_link_idx_in_parent_chain = chain_wrapper.parent_link_idx_in_parent_chain;
                let parent_macro_joint_idx = chain_wrapper.parent_macro_joint_idx;

                if let (Some(parent_chain_idx), Some(parent_link_idx_in_parent_chain), Some(parent_macro_joint_idx)) = (parent_chain_idx, parent_link_idx_in_parent_chain, parent_macro_joint_idx) {
                    let parent_link_name = chain_and_link_idx_to_new_link_name_mapping.get(&(parent_chain_idx, parent_link_idx_in_parent_chain)).expect("error");
                    let child_link_name = chain_and_link_idx_to_new_link_name_mapping.get(&(*chain_idx, chain_base_link_idx)).expect("error");
                    let joint = &self.macro_joints[parent_macro_joint_idx];

                    let new_joint = OJoint::new_manual(&format!("macro_joint_{}", parent_macro_joint_idx), joint.joint_type().clone(), joint.origin().pose.clone(), joint.axis().clone(), parent_link_name, child_link_name, joint.limit().clone(), joint.dynamics().clone(), joint.mimic().clone(), joint.safety_controller().clone());
                    joints.push(new_joint);
                }

                chain.joints().iter().for_each(|joint| {
                    let parent_link_name = chain_and_link_idx_to_new_link_name_mapping.get( &(*chain_idx, joint.parent_link_idx) ).expect("error");
                    let child_link_name = chain_and_link_idx_to_new_link_name_mapping.get( &(*chain_idx, joint.child_link_idx) ).expect("error");

                    let joint_name = format!("chain_{}_{}", chain_idx, joint.name());

                    let mut new_joint = joint.clone();
                    new_joint.name = joint_name;
                    new_joint.parent_link = parent_link_name.clone();
                    new_joint.child_link = child_link_name.clone();

                    joints.push(new_joint);
                });
            });
        });

        OChain::from_manual_no_mesh_setup("robot_as_chain", links, joints)
    }
    fn setup(&mut self) {
        self.set_chain_info();
        self.set_num_dofs();
        self.set_all_sub_dof_idxs();
        self.set_dof_to_joint_and_sub_dof_idxs();
        self.set_robot_as_single_chain();
    }
}
impl<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait> ORobot<T, C, L> {
    fn set_chain_info(&mut self) {
        let chain_info = compute_chain_info(&self.chain_wrappers, &self.macro_joints);

        self.base_chain_idx = chain_info.base_link_idx;
        self.kinematic_hierarchy_of_chains = chain_info.kinematic_hierarchy.clone();

        self.chain_wrappers.iter_mut().enumerate().for_each(|(i, x)| {
            x.parent_macro_joint_idx = chain_info.link_parent_joint[i].clone();
            if let Some(link_parent_joint_idx) = &chain_info.link_parent_joint[i] {
                let macro_joint = self.macro_joints[*link_parent_joint_idx].clone();
                x.parent_link_idx_in_parent_chain = Some(macro_joint.parent_link_idx_in_parent_chain);
            }
            x.children_macro_joint_idxs = chain_info.link_children_joints[i].clone();
            x.parent_chain_idx = chain_info.link_parent_link[i].clone();
            x.children_chain_idxs = chain_info.link_children_links[i].clone();
            x.chain_connection_paths = chain_info.link_connection_paths[i].clone();
        });
    }
    fn set_num_dofs(&mut self) {
        let mut num_dofs = 0;
        self.chain_wrappers.iter().for_each(|x| num_dofs += x.chain.num_dofs());
        self.macro_joints.iter().for_each(|x| num_dofs += x.get_num_dofs());

        self.num_dofs = num_dofs;
    }
    fn set_all_sub_dof_idxs(&mut self) {
        let mut count = 0;

        let mut macro_joint_sub_dof_idxs = vec![vec![]; self.macro_joints.len()];
        let mut macro_joint_sub_dof_idxs_range = vec![None; self.macro_joints.len()];
        let mut chain_sub_dof_idxs = vec![vec![]; self.chain_wrappers.len()];
        let mut chain_sub_dof_idxs_range = vec![None; self.chain_wrappers.len()];

        let kinematic_hierarchy = &self.kinematic_hierarchy_of_chains;
        for layer in kinematic_hierarchy {
            for chain_idx in layer {
                // let parent_joint = self.chain_info.link_parent_joint(*chain_idx);
                let parent_joint = self.chain_wrappers[*chain_idx].parent_macro_joint_idx;
                match parent_joint {
                    None => {}
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
        let mut dof_to_joint_and_sub_dof_idxs = vec![RobotJointIdx::default(); self.num_dofs];

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
    fn set_robot_as_single_chain(&mut self) {
        self.robot_as_single_chain = self.get_robot_as_single_chain();
    }
}
impl<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> AsChainTrait<T, C, L> for ORobot<T, C, L> {
    #[inline(always)]
    fn as_chain(&self) -> &OChain<T, C, L> {
        &self.robot_as_single_chain
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OChainWrapper<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> {
    #[serde(deserialize_with = "OChain::<T, C, L>::deserialize")]
    chain: OChain<T, C, L>,
    chain_idx: usize,
    pub(crate) parent_macro_joint_idx: Option<usize>,
    pub(crate) children_macro_joint_idxs: Vec<usize>,
    pub(crate) parent_chain_idx: Option<usize>,
    pub(crate) parent_link_idx_in_parent_chain: Option<usize>,
    pub(crate) children_chain_idxs: Vec<usize>,
    pub(crate) chain_connection_paths: Vec<Option<Vec<usize>>>,
    dof_idxs: Vec<usize>,
    dof_idxs_range: Option<(usize, usize)>,
}
impl<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait> OChainWrapper<T, C, L> {
    fn new_world_chain() -> Self {
        Self {
            chain: OChain::new_world_chain(),
            chain_idx: 0,
            parent_macro_joint_idx: None,
            children_macro_joint_idxs: vec![],
            parent_chain_idx: None,
            parent_link_idx_in_parent_chain: None,
            children_chain_idxs: vec![],
            chain_connection_paths: vec![],
            dof_idxs: vec![],
            dof_idxs_range: None,
        }
    }
    #[inline(always)]
    pub fn chain(&self) -> &OChain<T, C, L> {
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
    pub fn chain_connection_paths(&self) -> &Vec<Option<Vec<usize>>> {
        &self.chain_connection_paths
    }
    #[inline(always)]
    pub fn parent_macro_joint_idx(&self) -> &Option<usize> {
        &self.parent_macro_joint_idx
    }
    #[inline(always)]
    pub fn children_macro_joint_idxs(&self) -> &Vec<usize> {
        &self.children_macro_joint_idxs
    }
    #[inline(always)]
    pub fn parent_chain_idx(&self) -> &Option<usize> {
        &self.parent_chain_idx
    }
    #[inline(always)]
    pub fn children_chain_idxs(&self) -> &Vec<usize> {
        &self.children_chain_idxs
    }
    #[inline(always)]
    pub fn dof_idxs(&self) -> &Vec<usize> {
        &self.dof_idxs
    }
    #[inline(always)]
    pub fn dof_idxs_range(&self) -> &Option<(usize, usize)> {
        &self.dof_idxs_range
    }
}
impl<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait> OChain<T, C, L> {
    pub(crate) fn new_world_chain() -> Self {
        Self::from_manual("world", vec![OLink::new_manual("world_link", vec![], vec![], OInertial::new_zeros())], vec![])
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotFKResult<T: AD, P: O3DPose<T>> {
    #[serde(deserialize_with = "Vec::<Option::<ChainFKResult<T, P>>>::deserialize")]
    pub(crate) chain_fk_results: Vec<Option<ChainFKResult<T, P>>>,
    _phantom_data: PhantomData<T>,
}

impl<T: AD, P: O3DPose<T>> RobotFKResult<T, P> {
    #[inline]
    pub fn chain_fk_results(&self) -> &Vec<Option<ChainFKResult<T, P>>> {
        &self.chain_fk_results
    }
    #[inline]
    pub fn get_chain_fk_result(&self, chain_idx: usize) -> &Option<ChainFKResult<T, P>> {
        &self.chain_fk_results[chain_idx]
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum RobotJointIdx {
    ChainJoint { chain_idx: usize, joint_idx: usize, sub_dof_idx: usize },
    MacroJoint { macro_joint_idx: usize, sub_dof_idx: usize },
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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RobotLinkAuxInfo {
    chain_idx: usize,
    link_idx_in_chain: usize
}