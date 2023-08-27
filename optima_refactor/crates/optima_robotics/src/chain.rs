use std::collections::HashMap;
use std::fmt::Debug;
use std::marker::PhantomData;
use ad_trait::*;
use nalgebra::{Isometry3};
use serde::{Serialize, Deserialize};
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_utils::arr_storage::*;
use crate::utils::get_urdf_path_from_chain_name;
use serde_with::*;
use optima_3d_mesh::{SaveToSTL};
use optima_file::path::{OAssetLocation, OPath, OPathMatchingPattern, OPathMatchingStopCondition, OStemCellPath};
use optima_linalg::vecs_and_mats::{NalgebraLinalg, OLinalgTrait, OVec};
use crate::robotics_components::*;
use crate::robotics_traits::{ChainableTrait, JointTrait};

pub type OChainDefault<T> = OChain<T, Isometry3<T>, NalgebraLinalg>;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OChain<T: AD, P: O3DPose<T>, L: OLinalgTrait> {
    chain_name: String,
    #[serde(deserialize_with = "Vec::<OLink<T, P, L>>::deserialize")]
    links: Vec<OLink<T, P, L>>,
    #[serde(deserialize_with = "Vec::<OJoint<T, P>>::deserialize")]
    joints: Vec<OJoint<T, P>>,
    link_name_to_link_idx_map: HashMap<String, usize>,
    joint_name_to_joint_idx_map: HashMap<String, usize>,
    base_link_idx: usize,
    kinematic_hierarchy: Vec<Vec<usize>>,
    num_dofs: usize,
    dof_to_joint_and_sub_dof_idxs: Vec<(usize, usize)>,
    phantom_data: PhantomData<(T, P)>
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> OChain<T, P, L> {
    pub fn from_urdf(chain_name: &str) -> Self {
        let urdf_path = get_urdf_path_from_chain_name(chain_name);
        let urdf = urdf_path.load_urdf();

        let mut links = vec![];
        let mut joints = vec![];

        urdf.links.iter().for_each(|x| {
            links.push(OLink::from_link(x));
        });

        urdf.joints.iter().for_each(|x| {
            joints.push(OJoint::from_joint(x));
        });

        Self::from_manual(chain_name, links, joints)
    }
    pub fn from_manual(chain_name: &str, links: Vec<OLink<T, P, L>>, joints: Vec<OJoint<T, P>>) -> Self {
        let mut link_name_to_link_idx_map = HashMap::new();
        let mut joint_name_to_joint_idx_map = HashMap::new();

        links.iter().enumerate().for_each(|(i, x)| {
            link_name_to_link_idx_map.insert( x.name().to_string(), i );
        });

        joints.iter().enumerate().for_each(|(i, x)| {
            joint_name_to_joint_idx_map.insert( x.name().to_string(), i );
        });

        let mut out = Self {
            chain_name: chain_name.into(),
            links,
            joints,
            link_name_to_link_idx_map,
            joint_name_to_joint_idx_map,
            base_link_idx: usize::default(),
            kinematic_hierarchy: vec![],
            num_dofs: usize::default(),
            dof_to_joint_and_sub_dof_idxs: vec![],
            phantom_data: Default::default(),
        };

        out.setup();

        out
    }
    #[inline(always)]
    pub fn get_link_idx_from_link_name(&self, link_name: &str) -> usize {
        *self.link_name_to_link_idx_map.get(link_name).expect("error")
    }
    #[inline(always)]
    pub fn get_joint_idx_from_joint_name(&self, joint_name: &str) -> usize {
        *self.joint_name_to_joint_idx_map.get(joint_name).expect("error")
    }
    #[inline(always)]
    pub fn num_dofs(&self) -> usize {
        self.num_dofs
    }
    /*
    pub fn set_joint_as_fixed(&mut self, joint_idx: usize, fixed_values: &[T]) {
        let joint = self.joints.get_element_mut(joint_idx);

        if joint.mimic().is_some() { panic!("cannot fix joint values on a mimic joint.") }

        assert_eq!(fixed_values.len(), joint.joint_type().num_dofs());

        joint.fixed_values = Some(fixed_values.into());

        self.set_num_dofs();
        self.set_all_sub_dof_idxs();
    }
    pub fn set_dead_end_link(&mut self, link_idx: usize) {
        self.links[link_idx].is_present_in_model = false;

        let mut link_idx_stack = vec![ link_idx ];

        while !link_idx_stack.is_empty() {
            let link_idx = link_idx_stack.pop().unwrap();
            self.links.get_element_mut(link_idx).is_present_in_model = false;
            let parent_joint_idx = self.links.get_element(link_idx).parent_joint_idx();
            if let Some(parent_joint_idx) = parent_joint_idx {
                self.joints[*parent_joint_idx].is_present_in_model = false;
            }
            self.links.get_element(link_idx).children_link_idxs().iter().for_each(|x| link_idx_stack.push(*x));
        }

        self.set_num_dofs();
        self.set_all_sub_dof_idxs();
    }
    */
    #[inline]
    pub fn get_joint_transform<V: OVec<T>>(&self, state: &V, joint_idx: usize) -> P {
        self.joints[joint_idx].get_joint_transform(state, &self.joints)
    }
    #[inline(always)]
    pub fn get_joint_fixed_offset_transform(&self, joint_idx: usize) -> &P {
        self.joints[joint_idx].get_joint_fixed_offset_transform()
    }
    #[inline]
    pub fn get_joint_variable_transform<V: OVec<T>>(&self, state: &V, joint_idx: usize) -> P {
        self.joints[joint_idx].get_joint_variable_transform(state, &self.joints)
    }
    #[inline]
    pub fn dof_to_joint_and_sub_dof_idxs(&self) -> &Vec<(usize, usize)> {
        &self.dof_to_joint_and_sub_dof_idxs
    }
    pub fn forward_kinematics<V: OVec<T>>(&self, state: &V, base_offset: Option<&P>) -> ChainFKResult<T, P> {
        let mut out = vec![ None; self.links.len() ];

        let base_pose = match base_offset {
            None => { P::identity() }
            Some(base_offset) => { base_offset.to_owned() }
        };

        self.kinematic_hierarchy.iter().enumerate().for_each(|(layer_idx, layer)| {
            layer.iter().for_each(|link_idx| {
                if layer_idx == 0 { out[*link_idx] = Some(base_pose.clone()) }
                else {
                    let link = &self.links[*link_idx];
                    let parent_link_idx = link.parent_link_idx.unwrap();
                    let parent_joint_idx = link.parent_joint_idx.unwrap();
                    let transform = self.get_joint_transform(state, parent_joint_idx);
                    let new_pose = out[parent_link_idx].as_ref().unwrap().mul(&transform);
                    out[*link_idx] = Some(new_pose);
                }
            });
        });

        ChainFKResult { link_poses: out, _phantom_data: Default::default() }
    }
    pub fn forward_kinematics_floating_chain<V: OVec<T>>(&self, state: &V, start_link_idx: usize, end_link_idx: usize, base_offset: Option<&P>) -> ChainFKResult<T, P> {
        let mut out = vec![ None; self.links.len() ];

        let base_pose = match base_offset {
            None => { P::identity() }
            Some(base_offset) => { base_offset.to_owned() }
        };

        let link_chain = &self.links[start_link_idx].link_connection_paths[end_link_idx];
        match link_chain {
            None => {}
            Some(link_chain) => {
                link_chain.iter().enumerate().for_each(|(i, link_idx)| {
                    if i == 0 { out[*link_idx] = Some(base_pose.clone()); }
                    else {
                        let link = &self.links[*link_idx];
                        let parent_link_idx = link.parent_link_idx.unwrap();
                        let parent_joint_idx = link.parent_joint_idx.unwrap();
                        let transform = self.get_joint_transform(state, parent_joint_idx);
                        let new_pose = out[parent_link_idx].as_ref().unwrap().mul(&transform);
                        out[*link_idx] = Some(new_pose);
                    }
                });
            }
        }

        ChainFKResult { link_poses: out, _phantom_data: Default::default() }
    }
    fn setup(&mut self) {
        self.set_link_and_joint_idxs();
        self.assign_joint_connection_indices();
        self.set_mimic_joint_idxs();
        self.set_chain_info();
        self.set_num_dofs();
        self.set_all_sub_dof_idxs();
        self.set_dof_to_joint_and_sub_dof_idxs();
        self.set_link_original_mesh_file_paths();
        self.set_link_stl_mesh_file_paths();
    }
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> OChain<T, P, L> {
    fn set_link_and_joint_idxs(&mut self) {
        self.links.iter_mut().enumerate().for_each(|(i, x)| {
           x.link_idx = i;
        });

        self.joints.iter_mut().enumerate().for_each(|(i, x)| {
           x.joint_idx = i;
        });
    }
    fn assign_joint_connection_indices(&mut self) {
        for joint_idx in 0..self.joints.len() {
            self.joints.get_element_mut(joint_idx).parent_link_idx = self.get_link_idx_from_link_name(&self.joints.get_element(joint_idx).parent_link());
            self.joints.get_element_mut(joint_idx).child_link_idx = self.get_link_idx_from_link_name(&self.joints.get_element(joint_idx).child_link())
        }
    }
    fn set_mimic_joint_idxs(&mut self) {
        for joint_idx in 0..self.joints.len() {
            if let Some(mimic) = &self.joints[joint_idx].mimic() {
                let idx = self.get_joint_idx_from_joint_name(&mimic.joint());
                self.joints[joint_idx].mimic.as_mut().unwrap().joint_idx = idx;
            }
        }
    }
    fn set_chain_info(&mut self) {
        let chain_info = self.compute_chain_info();

        self.base_link_idx = chain_info.base_link_idx;
        self.kinematic_hierarchy = chain_info.kinematic_hierarchy.clone();

        self.links.iter_mut().enumerate().for_each(|(i, x)| {
            x.parent_joint_idx = chain_info.link_parent_joint[i].clone();
            x.children_joint_idxs = chain_info.link_children_joints[i].clone();
            x.parent_link_idx = chain_info.link_parent_link[i].clone();
            x.children_link_idxs = chain_info.link_children_links[i].clone();
            x.link_connection_paths = chain_info.link_connection_paths[i].clone();
        });
    }
    fn set_num_dofs(&mut self) {
        let mut num_dofs = 0;
        self.joints.iter().for_each(|x| num_dofs += x.get_num_dofs());
        self.num_dofs = num_dofs;
    }
    /*
    fn set_all_sub_dof_idxs(&mut self) {
        let mut count = 0;
        for joint_idx in 0..self.joints().len() {
            let num_dofs = self.joints().get_element(joint_idx).get_num_dofs();
            let joint = self.joints.get_element_mut(joint_idx);
            joint.sub_dof_idxs = ArrayVec::new();
            for _ in 0..num_dofs {
                joint.sub_dof_idxs.push(count);
                count += 1;
            }
        }

        for joint_idx in 0..self.joints().len() {
            let joint = self.joints.get_element_mut(joint_idx);
            joint.sub_dof_idxs_range = None;
            if joint.sub_dof_idxs.len() > 0 {
                let range_start = joint.sub_dof_idxs.get_element(0);
                let range_end = joint.sub_dof_idxs.get_element(joint.sub_dof_idxs.len()-1);
                joint.sub_dof_idxs_range = Some( (*range_start, *range_end + 1) );
            }
        }
    }
    */
    fn set_all_sub_dof_idxs(&mut self) {
        let mut count = 0;

        let mut joint_sub_dof_idxs = vec![];
        let mut joint_sub_dof_idxs_range = vec![];

        for joint_idx in 0..self.joints().len() {
            let num_dofs = self.joints().get_element(joint_idx).get_num_dofs();
            let mut sub_dof_idxs = vec![];
            for _ in 0..num_dofs {
                sub_dof_idxs.push(count);
                count += 1;
            }
            joint_sub_dof_idxs.push(sub_dof_idxs);
        }
        for joint_idx in 0..self.joints().len() {
            let mut sub_dof_idxs_range = None;
            if joint_sub_dof_idxs[joint_idx].len() > 0 {
                let range_start = joint_sub_dof_idxs[joint_idx].first().unwrap();
                let range_end = joint_sub_dof_idxs[joint_idx].last().unwrap();
                sub_dof_idxs_range = Some( (*range_start, *range_end + 1) );
            }
            joint_sub_dof_idxs_range.push(sub_dof_idxs_range);
        }

        joint_sub_dof_idxs.iter().zip(joint_sub_dof_idxs_range.iter()).enumerate().for_each(|(i, (x,y))| {
            self.joints[i].dof_idxs = x.clone();
            self.joints[i].dof_idxs_range = y.clone();
        })
    }
    fn set_dof_to_joint_and_sub_dof_idxs(&mut self) {
        let mut dof_to_joint_and_sub_dof_idxs = vec![];

        self.joints.iter().enumerate().for_each(|(joint_idx, joint)| {
           joint.dof_idxs.iter().enumerate().for_each(|(i, _)| {
               dof_to_joint_and_sub_dof_idxs.push( (joint_idx, i) );
           })
        });

        self.dof_to_joint_and_sub_dof_idxs = dof_to_joint_and_sub_dof_idxs;
    }
    fn set_link_original_mesh_file_paths(&mut self) {
        self.links.iter_mut().for_each(|link| {
            if link.visual().len() > 0 {
                let geometry = link.visual()[0].geometry().clone();
                match geometry {
                    OGeometry::Mesh { filename, .. } => {
                        let split = filename.split("//");
                        let split: Vec<String> = split.map(|x| x.to_string()).collect();
                        let filepath = split.last().unwrap().to_owned();
                        let split = filepath.split("/");
                        let split: Vec<String> = split.map(|x| x.to_string()).collect();

                        let file_check = split.last().unwrap().to_owned();
                        let mut target_path = OStemCellPath::new_asset_path();
                        target_path.append_file_location(&OAssetLocation::ChainOriginalMeshes { chain_name: &self.chain_name });
                        target_path.append(&file_check);
                        let exists = target_path.exists();

                        if !exists {
                            let asset_path = OPath::new_home_path();
                            println!("searching for mesh {:?}", filepath);
                            let found_paths = asset_path.walk_directory_and_match(OPathMatchingPattern::PathComponents(split), OPathMatchingStopCondition::First);
                            if found_paths.is_empty() {
                                panic!("could not find filepath for link mesh: {:?}", filename);
                            }

                            let found_path = found_paths[0].clone();
                            found_path.copy_file_to_destination(target_path.as_physical_path()).expect("error: file could not be copied.");
                        }

                        link.original_mesh_file_path = Some(target_path.clone());
                    }
                    _ => {}
                }
            }
        });
    }
    fn set_link_stl_mesh_file_paths(&mut self) {
        self.links.iter_mut().for_each(|link| {
            let original_mesh_file_path = &link.original_mesh_file_path;
            if let Some(original_mesh_file_path) = original_mesh_file_path {
                let extension = original_mesh_file_path.extension().expect("must have extension");
                let filename = original_mesh_file_path.filename_without_extension().expect("must have filename");
                let mut target_path = OStemCellPath::new_asset_path();
                target_path.append_file_location(&OAssetLocation::ChainSTLMeshes { chain_name: &self.chain_name });
                target_path.append(&(filename.clone() + ".stl"));
                let exists = target_path.exists();

                if !exists {
                    if extension.as_str() == "stl" || extension.as_str() == "STL" {
                        original_mesh_file_path.load_stl().save_to_stl(&target_path);
                    } else if extension.as_str() == "dae" || extension.as_str() == "DAE" {
                        original_mesh_file_path.load_dae().save_to_stl(&target_path);
                    } else {
                        panic!("extension {} is unsupported.", extension);
                    };
                }

                link.stl_mesh_file_path = Some(target_path.clone());
            }
        });
    }
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ChainableTrait<T, P> for OChain<T, P, L> {
    type LinkType = OLink<T, P, L>;
    type JointType = OJoint<T, P>;

    #[inline(always)]
    fn links(&self) -> &Vec<Self::LinkType> {
        &self.links
    }

    #[inline(always)]
    fn joints(&self) -> &Vec<Self::JointType> {
        &self.joints
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChainFKResult<T: AD, P: O3DPose<T>> {
    #[serde(deserialize_with = "Vec::<Option::<P>>::deserialize")]
    pub (crate) link_poses: Vec<Option<P>>,
    _phantom_data: PhantomData<T>
}
impl<T: AD, P: O3DPose<T>> ChainFKResult<T, P> {
    #[inline]
    pub fn link_poses(&self) -> &Vec<Option<P>> {
        &self.link_poses
    }
    #[inline]
    pub fn get_link_pose(&self, link_idx: usize) -> &Option<P> {
        &self.link_poses[link_idx]
    }
}

/*
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> OForwardKinematicsTrait<T, P> for OChain<T, P, L> {
    fn forward_kinematics<V: OVec<T>>(&self, state: &V, base_offset: Option<&P>) -> Vec<Vec<Option<P>>> {
        assert_eq!(state.len(), self.num_dofs);

        let mut out = vec![ None; self.links.len() ];

        let base_pose = match base_offset {
            None => { P::identity() }
            Some(base_offset) => { base_offset.clone() }
        };

        for (layer_idx, link_kinematic_hierarchy_layer) in self.link_kinematic_hierarchy().iter().enumerate() {
            'l: for link_idx in link_kinematic_hierarchy_layer {
                let link = self.links().get_element(*link_idx);
                if !link.is_present_in_model { continue 'l; }

                if layer_idx == 0 {
                    out[*link_idx] = Some( base_pose.clone() )
                } else {
                    let parent_link_idx = link.parent_link_idx.unwrap();
                    let parent_joint_idx = link.parent_joint_idx.unwrap();

                    let joint_transform = self.get_joint_transform(state, parent_joint_idx);
                    let link_pose = out[parent_link_idx].as_ref().unwrap().mul(&joint_transform);
                    out[*link_idx] = Some(link_pose);
                }
            }
        }

        vec![out]
    }

    fn forward_kinematics_floating_chain<V: OVec<T>>(&self, state: &V, start_link_idx: usize, end_link_idx: usize, base_offset: Option<&P>) -> Vec<Vec<Option<P>>> {
        let link_path = self.links[start_link_idx].link_idx_path_to_other_link(end_link_idx);
        assert!(link_path.is_some());
        let link_path = link_path.as_ref().unwrap();

        assert_eq!(state.len(), self.num_dofs);

        let mut out = vec![ None; self.links.len() ];

        let base_pose = match base_offset {
            None => { P::identity() }
            Some(base_offset) => { base_offset.clone() }
        };

        'l: for (i, link_idx) in link_path.iter().enumerate() {
            let link = self.links().get_element(*link_idx);
            if !link.is_present_in_model { continue 'l; }

            if i == 0 {
                out[*link_idx] = Some( base_pose.clone() )
            } else {
                let parent_link_idx = link.parent_link_idx.unwrap();
                let parent_joint_idx = link.parent_joint_idx.unwrap();

                let joint_transform = self.get_joint_transform(state, parent_joint_idx);
                let link_pose = out[parent_link_idx].as_ref().unwrap().mul(&joint_transform);
                out[*link_idx] = Some(link_pose);
            }
        }

        vec![out]
    }
}
*/
/*
// Robot Storage Trait //

pub trait ORobotArrStorageTrait: Clone + Debug + DeserializeOwned + Serialize {
    type LinkKinHierarchyLayersArrType: ArrStorageTrait;
    type LinkKinHierarchySingleLayerArrType: ArrStorageTrait;
    type LinkIdxPathsArrType: ArrStorageTrait;
    type LinksArrType: ArrStorageTrait;
    type JointsArrType: ArrStorageTrait;
    type SuccessorJointIdxsArrType: ArrStorageTrait;
    type SuccessorLinkIdxsArrType: ArrStorageTrait;
    type LinkNameStrType: StrStorageTrait;
    type CollisionArrType: ArrStorageTrait;
    type VisualArrType: ArrStorageTrait;
    type JointNameStrType: StrStorageTrait;
    type ParentLinkStrType: StrStorageTrait;
    type ChildLinkStrType: StrStorageTrait;
    type CollisionNameStrType: StrStorageTrait;
    type VisualNameStrType: StrStorageTrait;
    type MaterialNameStrType: StrStorageTrait;
    type TextureFilenameStrType: StrStorageTrait;
    type GeometryFilenameStrType: StrStorageTrait;
    type MimicJointNameStrType: StrStorageTrait;
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ORobotArrStorageVec;
impl ORobotArrStorageTrait for ORobotArrStorageVec {
    type LinkKinHierarchyLayersArrType = VecStor;
    type LinkKinHierarchySingleLayerArrType = VecStor;
    type LinkIdxPathsArrType = VecStor;
    type LinksArrType = VecStor;
    type JointsArrType = VecStor;
    type SuccessorJointIdxsArrType = VecStor;
    type SuccessorLinkIdxsArrType = VecStor;
    type LinkNameStrType = StringStor;
    type CollisionArrType = VecStor;
    type VisualArrType = VecStor;
    type JointNameStrType = StringStor;
    type ParentLinkStrType = StringStor;
    type ChildLinkStrType = StringStor;
    type CollisionNameStrType = StringStor;
    type VisualNameStrType = StringStor;
    type MaterialNameStrType = StringStor;
    type TextureFilenameStrType = StringStor;
    type GeometryFilenameStrType = StringStor;
    type MimicJointNameStrType = StringStor;
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ORobotArrStorageArrayVec<
    const NUM_LINK_HIERARCHY_TREE_LAYERS: usize,
    const NUM_LINKS_PER_KIN_HIERARCHY_LAYERS: usize,
    const NUM_LINKS: usize,
    const NUM_JOINTS: usize,
    const NAME_STRING_LEN: usize
>;
impl<
    const NUM_LINK_KIN_HIERARCHY_LAYERS: usize,
    const NUM_LINKS_PER_KIN_HIERARCHY_LAYERS: usize,
    const NUM_LINKS: usize, const NUM_JOINTS: usize,
    const NAME_STRING_LEN: usize
>
ORobotArrStorageTrait for
ORobotArrStorageArrayVec<
    NUM_LINK_KIN_HIERARCHY_LAYERS,
    NUM_LINKS_PER_KIN_HIERARCHY_LAYERS,
    NUM_LINKS,
    NUM_JOINTS,
    NAME_STRING_LEN
>
{
    type LinkKinHierarchyLayersArrType = ArrayVecStor<NUM_LINK_KIN_HIERARCHY_LAYERS>;
    type LinkKinHierarchySingleLayerArrType = ArrayVecStor<NUM_LINKS_PER_KIN_HIERARCHY_LAYERS>;
    type LinkIdxPathsArrType = ArrayVecStor<NUM_LINKS>;
    type LinksArrType = VecStor;
    type JointsArrType = VecStor;
    type SuccessorJointIdxsArrType = ArrayVecStor<15>;
    type SuccessorLinkIdxsArrType = ArrayVecStor<15>;
    type LinkNameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type CollisionArrType = ArrayVecStor<2>;
    type VisualArrType = ArrayVecStor<2>;
    type JointNameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type ParentLinkStrType = ArrayStringStor<NAME_STRING_LEN>;
    type ChildLinkStrType = ArrayStringStor<NAME_STRING_LEN>;
    type CollisionNameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type VisualNameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type MaterialNameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type TextureFilenameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type GeometryFilenameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type MimicJointNameStrType = ArrayStringStor<NAME_STRING_LEN>;
}
*/
