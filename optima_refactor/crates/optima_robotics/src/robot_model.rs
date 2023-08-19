use std::collections::HashMap;
use std::fmt::Debug;
use std::marker::PhantomData;
use ad_trait::*;
use nalgebra::Isometry3;
use urdf_rs::{Collision, Color, Dynamics, Geometry, Inertial, Joint, JointLimit, JointType, Link, Material, Mimic, Pose, SafetyController, Texture, Visual};
use serde::{Serialize, Deserialize};
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_utils::arr_storage::*;
use crate::utils::get_urdf_path_from_robot_name;
use serde_with::*;
use optima_3d_spatial::optima_3d_pose::SerdeO3DPose;
use optima_linalg::vecs_and_mats::{NalgebraLinalg, OLinalgTrait, OMat};
use optima_linalg::vecs_and_mats::SerdeOMat;
use optima_utils::arr_storage::ImmutArrTrait;

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

// Robot Model //

pub type ORobotModelDefault<T> = ORobotModel<T, Isometry3<T>, NalgebraLinalg>;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ORobotModel<T: AD, P: O3DPose<T>, L: OLinalgTrait> {
    #[serde(deserialize_with = "Vec::<OLink<T, P, L>>::deserialize")]
    links: Vec<OLink<T, P, L>>,
    #[serde(deserialize_with = "Vec::<OJoint<T, P>>::deserialize")]
    joints: Vec<OJoint<T, P>>,
    link_name_to_link_idx_map: HashMap<String, usize>,
    joint_name_to_joint_idx_map: HashMap<String, usize>,
    link_kinematic_hierarchy: Vec<Vec<usize>>,
    base_link_idx: usize,
    phantom_data: PhantomData<(T, P)>
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ORobotModel<T, P, L> {
    pub fn from_urdf(robot_name: &str) -> Self {
        let urdf_path = get_urdf_path_from_robot_name(robot_name);
        let urdf = urdf_path.load_urdf();

        let mut links = vec![];
        let mut joints = vec![];

        let mut link_name_to_link_idx_map = HashMap::new();

        urdf.links.iter().enumerate().for_each(|(i, x)| {
            links.push(OLink::from_link(x, i));
            link_name_to_link_idx_map.insert( x.name.clone(), i );
        });

        let mut joint_name_to_joint_idx_map = HashMap::new();

        urdf.joints.iter().enumerate().for_each(|(i, x)| {
            joints.push(OJoint::from_joint(x, i));
            joint_name_to_joint_idx_map.insert( x.name.clone(), i );
        });

        let mut out = Self {
            links: Vec::from_slice(&links),
            joints: Vec::from_slice(&joints),
            link_name_to_link_idx_map,
            joint_name_to_joint_idx_map,
            link_kinematic_hierarchy: Vec::default(),
            base_link_idx: usize::default(),
            phantom_data: Default::default(),
        };

        out.setup();

        out
    }
    pub fn get_link_idx_from_link_name(&self, link_name: &str) -> usize {
        *self.link_name_to_link_idx_map.get(link_name).expect("error")
    }
    pub fn get_joint_idx_from_joint_name(&self, joint_name: &str) -> usize {
        *self.joint_name_to_joint_idx_map.get(joint_name).expect("error")
    }
    pub fn base_link_idx(&self) -> usize {
        self.base_link_idx
    }
    pub fn links(&self) -> &Vec<OLink<T, P, L>> {
        &self.links
    }
    pub fn joints(&self) -> &Vec<OJoint<T, P>> {
        &self.joints
    }
    pub fn link_kinematic_hierarchy(&self) -> &Vec<Vec<usize>> {
        &self.link_kinematic_hierarchy
    }
    fn setup(&mut self) {
        self.assign_joint_connection_indices();
        self.assign_link_preceding_and_successor_joints();
        self.assign_link_preceding_and_successor_links();
        self.construct_link_kinematic_tree();
        self.assign_layer_idxs_in_kin_hierarchy();
        self.assign_base_link_idx();
        self.create_link_idx_paths_between_all_links();
    }
}
/// All setup functions
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> ORobotModel<T, P, L> {
    fn assign_joint_connection_indices(&mut self) {
        for joint_idx in 0..self.joints.len() {
            self.joints.get_element_mut(joint_idx).parent_link_idx = self.get_link_idx_from_link_name(&self.joints.get_element(joint_idx).parent_link.as_str());
            self.joints.get_element_mut(joint_idx).child_link_idx = self.get_link_idx_from_link_name(&self.joints.get_element(joint_idx).child_link.as_str())
        }
    }
    fn assign_link_preceding_and_successor_joints(&mut self) {
        for joint_idx in 0..self.joints.len() {
            let parent_link_idx = self.joints().get_element(joint_idx).parent_link_idx;
            let child_link_idx = self.joints().get_element(joint_idx).child_link_idx;

            self.links.get_element_mut(parent_link_idx).children_joint_idxs.push(joint_idx);
            self.links.get_element_mut(child_link_idx).parent_joint_idx = Some(joint_idx);
        }
    }
    fn assign_link_preceding_and_successor_links(&mut self) {
        for link_idx in 0..self.links.len() {
            let preceding_joint_idx = self.links().get_element(link_idx).parent_joint_idx;
            let successor_joint_idxs = self.links().get_element(link_idx).children_joint_idxs.clone();

            match preceding_joint_idx {
                None => {}
                Some(preceding_joint_idx) => { self.links.get_element_mut(link_idx).parent_link_idx = Some(self.joints.get_element(preceding_joint_idx).parent_link_idx); }
            }

            successor_joint_idxs.as_slice().iter().for_each(|x| {
                self.links.get_element_mut(link_idx).children_link_idxs.push( self.joints.get_element(*x).child_link_idx );
            });
        }
    }
    fn construct_link_kinematic_tree(&mut self) {
        /*
        let mut root_node_candidate = vec![true; self.links().len()];
        self.joints().as_slice().iter().for_each(|x| root_node_candidate[ x.child_link_idx ] = false);

        let mut root_node_idx: Option<usize> = None;
        root_node_candidate.iter().enumerate().for_each(|(i, x)| if *x {
            match root_node_idx {
                None => { root_node_idx = Some(i) }
                Some(_) => { panic!("multiple root nodes detected. {} and {}.", root_node_idx.unwrap(), i) }
            }
        });
        let root_node_idx = root_node_idx.unwrap();

        println!(" >> {}", root_node_idx);
        */

        let mut root_node_idx: Option<usize> = None;
        self.links().as_slice().iter().enumerate().for_each(|(i, x)| match &x.parent_joint_idx {
            None => {
                match root_node_idx {
                    None => { root_node_idx = Some(i) }
                    Some(_) => { panic!("multiple root nodes found.  {} and {}.", root_node_idx.unwrap(), i) }
                }
            }
            Some(_) => { }
        });
        let root_node_idx = root_node_idx.expect("no root node found");

        self.link_kinematic_hierarchy.push( vec![root_node_idx] );
        let mut curr_layer_idx = 0;

        'l: loop {
            let mut new_layer = Vec::new();
            self.links().as_slice().iter().for_each(|link| {
                match &link.parent_link_idx {
                    None => {}
                    Some(preceding_link_idx) => {
                        if self.link_kinematic_hierarchy.get_element(curr_layer_idx).contains_element(preceding_link_idx) {
                            new_layer.push(link.link_idx);
                        }
                    }
                }
            });
            if new_layer.len() == 0 { break 'l; } else {
                self.link_kinematic_hierarchy.push(new_layer);
                curr_layer_idx += 1;
            }
        }
    }
    fn assign_layer_idxs_in_kin_hierarchy(&mut self) {
        for link_idx in 0..self.links().len() {
            'l: for layer_idx in 0..self.link_kinematic_hierarchy().len() {
                if self.link_kinematic_hierarchy().get_element(layer_idx).contains_element(&link_idx) {
                    self.links.get_element_mut(link_idx).layer_idx_in_kinematic_hierarchy = layer_idx;
                    break 'l;
                }
            }
        }
    }
    fn assign_base_link_idx(&mut self) {
        self.base_link_idx = *self.link_kinematic_hierarchy.get_element(0).get_element(0);
    }
    fn create_link_idx_paths_between_all_links(&mut self) {
        for i in 0..self.links.len() { self.create_link_idx_paths_from_given_link(i); }
    }
    fn create_link_idx_paths_from_given_link(&mut self, link_idx: usize) {
        let mut link_idx_paths_to_other_links: Vec<Option<Vec<usize>>> = vec![];
        let link_hierarchy_layer = self.links().get_element(link_idx).layer_idx_in_kinematic_hierarchy;
        'l: for target_link_idx in 0..self.links.len() {
            let target_link_hierarchy_layer = self.links().get_element(target_link_idx).layer_idx_in_kinematic_hierarchy;
            if target_link_hierarchy_layer <= link_hierarchy_layer { link_idx_paths_to_other_links.push(None); continue 'l; }

            let mut path_stack = vec![ vec![link_idx] ];

            'l1: loop {
                if path_stack.is_empty() { link_idx_paths_to_other_links.push(None); continue 'l; }
                let curr_path = path_stack.pop().unwrap();
                let last_added_idx = curr_path[curr_path.len() - 1];
                let children_link_idxs = self.links().get_element(last_added_idx).children_link_idxs.clone();
                if children_link_idxs.is_empty() { continue 'l1; }
                else {
                    for child_link_idx in children_link_idxs {
                        let mut curr_path_copy = curr_path.clone();
                        curr_path_copy.push(child_link_idx);
                        if child_link_idx == target_link_idx { link_idx_paths_to_other_links.push(Some(curr_path_copy)); continue 'l; }
                        else { path_stack.push(curr_path_copy); }
                    }
                }
            }
        }
        self.links[link_idx].link_idx_paths_to_other_links = link_idx_paths_to_other_links;
    }
}

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

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

// Components //

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OLink<T: AD, P: O3DPose<T>, L: OLinalgTrait> {
    present_in_model: bool,
    link_idx: usize,
    name: String,
    layer_idx_in_kinematic_hierarchy: usize,
    parent_joint_idx: Option<usize>,
    children_joint_idxs: Vec<usize>,
    parent_link_idx: Option<usize>,
    children_link_idxs: Vec<usize>,
    link_idx_paths_to_other_links: Vec<Option<Vec<usize>>>,
    #[serde(deserialize_with = "Vec::<OCollision<T, P>>::deserialize")]
    collision: Vec<OCollision<T, P>>,
    #[serde(deserialize_with = "Vec::<OVisual<T, P>>::deserialize")]
    visual: Vec<OVisual<T, P>>,
    #[serde(deserialize_with = "OInertial::<T, L>::deserialize")]
    inertial: OInertial<T, L>
}
impl<T: AD, P: O3DPose<T>, L: OLinalgTrait> OLink<T, P, L> {
    fn from_link(link: &Link, link_idx: usize) -> Self {
        Self {
            present_in_model: true,
            link_idx,
            name: String::from(&link.name),
            layer_idx_in_kinematic_hierarchy: usize::default(),
            parent_joint_idx: None,
            children_joint_idxs: Vec::default(),
            parent_link_idx: None,
            children_link_idxs: Vec::default(),
            link_idx_paths_to_other_links: Vec::default(),
            collision: link.collision.iter().map(|x| OCollision::from_collision(x)).collect(),
            visual: link.visual.iter().map(|x| OVisual::from_visual(x)).collect(),
            inertial: OInertial::from_inertial(&link.inertial)
        }
    }
    pub fn present_in_model(&self) -> bool {
        self.present_in_model
    }
    pub fn link_idx(&self) -> usize {
        self.link_idx
    }
    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn layer_idx_in_kinematic_hierarchy(&self) -> usize {
        self.layer_idx_in_kinematic_hierarchy
    }
    pub fn parent_joint_idx(&self) -> &Option<usize> {
        &self.parent_joint_idx
    }
    pub fn children_joint_idxs(&self) -> &Vec<usize> {
        &self.children_joint_idxs
    }
    pub fn parent_link_idx(&self) -> &Option<usize> {
        &self.parent_link_idx
    }
    pub fn children_link_idxs(&self) -> &Vec<usize> {
        &self.children_link_idxs
    }
    pub fn collision(&self) -> &Vec<OCollision<T, P>> {
        &self.collision
    }
    pub fn visual(&self) -> &Vec<OVisual<T, P>> {
        &self.visual
    }
    pub fn inertial(&self) -> &OInertial<T, L> {
        &self.inertial
    }
    pub fn link_idx_paths_to_other_links(&self) -> &Vec<Option<Vec<usize>>> {
        &self.link_idx_paths_to_other_links
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OJoint<T: AD, P: O3DPose<T>> {
    joint_idx: usize,
    name: String,
    joint_type: OJointType,
    #[serde(deserialize_with = "OPose::<T, P>::deserialize")]
    origin: OPose<T, P>,
    #[serde_as(as = "[SerdeAD<T>; 3]")]
    axis: [T; 3],
    parent_link: String,
    parent_link_idx: usize,
    child_link: String,
    child_link_idx: usize,
    #[serde(deserialize_with = "OJointLimit::<T>::deserialize")]
    limit: OJointLimit<T>,
    #[serde(deserialize_with = "Option::<ODynamics::<T>>::deserialize")]
    dynamics: Option<ODynamics<T>>,
    #[serde(deserialize_with = "Option::<OMimic<T>>::deserialize")]
    mimic: Option<OMimic<T>>,
    #[serde(deserialize_with = "Option::<OSafetyController<T>>::deserialize")]
    safety_controller: Option<OSafetyController<T>>
}
impl<T: AD, P: O3DPose<T>> OJoint<T, P> {
    fn from_joint(joint: &Joint, joint_idx: usize) -> Self {
        let axis: Vec<T> = joint.axis.xyz.0.iter().map(|x| T::constant(*x)).collect();

        Self {
            joint_idx,
            name: String::from(&joint.name),
            joint_type: OJointType::from_joint_type(&joint.joint_type),
            origin: OPose::from_pose(&joint.origin),
            axis: [axis[0], axis[1], axis[2]],
            parent_link: String::from(&joint.parent.link),
            parent_link_idx: usize::default(),
            child_link: String::from(&joint.child.link),
            child_link_idx: usize::default(),
            limit: OJointLimit::from_joint_limit(&joint.limit),
            dynamics: match &joint.dynamics {
                None => { None }
                Some(dynamics) => { Some(ODynamics::from_dynamics(dynamics)) }
            },
            mimic: match &joint.mimic {
                None => { None }
                Some(mimic) => { Some(OMimic::from_mimic(mimic)) }
            },
            safety_controller: match &joint.safety_controller {
                None => { None }
                Some(safety_controller) => { Some(OSafetyController::from_safety_controller(safety_controller)) }
            }
        }
    }
    pub fn joint_idx(&self) -> usize {
        self.joint_idx
    }
    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn joint_type(&self) -> &OJointType {
        &self.joint_type
    }
    pub fn origin(&self) -> &OPose<T, P> {
        &self.origin
    }
    pub fn axis(&self) -> &[T; 3] {
        &self.axis
    }
    pub fn parent_link(&self) -> &str {
        &self.parent_link
    }
    pub fn parent_link_idx(&self) -> usize {
        self.parent_link_idx
    }
    pub fn child_link(&self) -> &str {
        &self.child_link
    }
    pub fn child_link_idx(&self) -> usize {
        self.child_link_idx
    }
    pub fn limit(&self) -> &OJointLimit<T> {
        &self.limit
    }
    pub fn dynamics(&self) -> &Option<ODynamics<T>> {
        &self.dynamics
    }
    pub fn mimic(&self) -> &Option<OMimic<T>> {
        &self.mimic
    }
    pub fn safety_controller(&self) -> &Option<OSafetyController<T>> {
        &self.safety_controller
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OInertial<T: AD, L: OLinalgTrait> {
    #[serde_as(as = "SerdeOMat<T, L::MatType<T>>")]
    inertial_matrix: L::MatType<T>,
    #[serde_as(as = "SerdeAD<T>")]
    ixx: T,
    #[serde_as(as = "SerdeAD<T>")]
    ixy: T,
    #[serde_as(as = "SerdeAD<T>")]
    ixz: T,
    #[serde_as(as = "SerdeAD<T>")]
    iyy: T,
    #[serde_as(as = "SerdeAD<T>")]
    iyz: T,
    #[serde_as(as = "SerdeAD<T>")]
    izz: T
}
impl<T: AD, L: OLinalgTrait> OInertial<T, L> {
    fn from_inertial(inertial: &Inertial) -> Self {
        let mat_slice = [
            T::constant(inertial.inertia.ixx),
            T::constant(inertial.inertia.ixy),
            T::constant(inertial.inertia.ixz),
            T::constant(inertial.inertia.ixy),
            T::constant(inertial.inertia.iyy),
            T::constant(inertial.inertia.iyz),
            T::constant(inertial.inertia.ixz),
            T::constant(inertial.inertia.iyz),
            T::constant(inertial.inertia.izz)
        ];

        let inertial_matrix = L::MatType::from_column_major_slice(&mat_slice, 3, 3);

        Self {
            inertial_matrix,
            ixx: T::constant(inertial.inertia.ixx),
            ixy: T::constant(inertial.inertia.ixy),
            ixz: T::constant(inertial.inertia.ixz),
            iyy: T::constant(inertial.inertia.iyy),
            iyz: T::constant(inertial.inertia.iyz),
            izz: T::constant(inertial.inertia.izz)
        }
    }
    pub fn inertial_matrix(&self) -> &L::MatType<T> {
        &self.inertial_matrix
    }
    pub fn ixx(&self) -> &T {
        &self.ixx
    }
    pub fn ixy(&self) -> &T {
        &self.ixy
    }
    pub fn ixz(&self) -> &T {
        &self.ixz
    }
    pub fn iyy(&self) -> &T {
        &self.iyy
    }
    pub fn iyz(&self) -> &T {
        &self.iyz
    }
    pub fn izz(&self) -> &T {
        &self.izz
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OCollision<T: AD, P: O3DPose<T>> {
    geometry: OGeometry,
    name: Option<String>,
    #[serde(deserialize_with = "OPose::<T, P>::deserialize")]
    origin: OPose<T, P>
}
impl<T: AD, P: O3DPose<T>> OCollision<T, P> {
    fn from_collision(collision: &Collision) -> Self {
        Self {
            geometry: OGeometry::from_geometry(&collision.geometry),
            name: match &collision.name {
                None => { None }
                Some(name) => { Some(String::from(name)) }
            },
            origin: OPose::from_pose(&collision.origin)
        }
    }
    pub fn geometry(&self) -> &OGeometry {
        &self.geometry
    }
    pub fn name(&self) -> &Option<String> {
        &self.name
    }
    pub fn origin(&self) -> &OPose<T, P> {
        &self.origin
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OVisual<T: AD, P: O3DPose<T>> {
    name: Option<String>,
    material: Option<OMaterial>,
    #[serde(deserialize_with = "OPose::<T, P>::deserialize")]
    origin: OPose<T, P>,
    #[serde(deserialize_with = "OGeometry::deserialize")]
    geometry: OGeometry
}
impl<T: AD, P: O3DPose<T>> OVisual<T, P> {
    fn from_visual(visual: &Visual) -> Self {
        Self {
            name: match &visual.name {
                None => { None }
                Some(name) => { Some(String::from(name)) }
            },
            material: match &visual.material {
                None => { None }
                Some(material) => { Some(OMaterial::from_material(material)) }
            },
            origin: OPose::from_pose(&visual.origin),
            geometry: OGeometry::from_geometry(&visual.geometry)
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OMaterial {
    #[serde(deserialize_with = "Option::<OTexture>::deserialize")]
    texture: Option<OTexture>,
    name: String,
    color: Option<OColor>
}
impl OMaterial {
    fn from_material(material: &Material) -> Self {
        Self {
            texture: match &material.texture {
                None => { None }
                Some(texture) => { Some(OTexture::from_texture(texture)) }
            },
            name: String::from(&material.name),
            color: match &material.color {
                None => { None }
                Some(color) => { Some(OColor::from_color(color)) }
            }
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OTexture {
    filename: String
}
impl OTexture {
    fn from_texture(texture: &Texture) -> Self {
        Self {
            filename: String::from(&texture.filename)
        }
    }
    pub fn filename(&self) -> &str {
        &self.filename
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OColor {
    rgba: [f64; 4]
}
impl OColor {
    fn from_color(color: &Color) -> Self {
        Self {
            rgba: color.rgba.0
        }
    }
    pub fn rgba(&self) -> &[f64; 4] {
        &self.rgba
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum OGeometry {
    Box { size: [f64; 3] },
    Cylinder { radius: f64, length: f64 },
    Capsule { radius: f64, length: f64 },
    Sphere { radius: f64 },
    Mesh { filename: String, scale: Option<[f64; 3]> }
}
impl OGeometry {
    fn from_geometry(geometry: &Geometry) -> Self {
        return match &geometry {
            Geometry::Box { size } => { OGeometry::Box { size: size.0 } }
            Geometry::Cylinder { radius, length } => { OGeometry::Cylinder { radius: *radius, length: *length } }
            Geometry::Capsule { radius, length } => { OGeometry::Capsule { radius: *radius, length: *length } }
            Geometry::Sphere { radius } => { OGeometry::Sphere { radius: *radius } }
            Geometry::Mesh { filename, scale } => { OGeometry::Mesh { filename: String::from(filename), scale: match scale { None => { None } Some(scale) => { Some(scale.0) } } } }
        }
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OPose<T: AD, P: O3DPose<T>> {
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    pose: P,
    #[serde_as(as = "[SerdeAD<T>; 3]")]
    rpy: [T; 3],
    #[serde_as(as = "[SerdeAD<T>; 3]")]
    xyz: [T; 3]
}
impl<T: AD, P: O3DPose<T>> OPose<T, P> {
    fn from_pose(pose: &Pose) -> Self {
        let rpy: Vec<T> = pose.rpy.0.iter().map(|x| T::constant(*x)).collect();
        let xyz: Vec<T> = pose.xyz.0.iter().map(|x| T::constant(*x)).collect();

        let pose = P::from_translation_and_rotation_constructor(&xyz, &rpy);

        Self {
            pose,
            rpy: [rpy[0], rpy[1], rpy[2]],
            xyz: [xyz[0], xyz[1], xyz[2]]
        }
    }
    pub fn pose(&self) -> &P {
        &self.pose
    }
    pub fn rpy(&self) -> &[T; 3] {
        &self.rpy
    }
    pub fn xyz(&self) -> &[T; 3] {
        &self.xyz
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum OJointType {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar,
    Spherical
}
impl OJointType {
    fn from_joint_type(joint_type: &JointType) -> Self {
        match joint_type {
            JointType::Revolute => { OJointType::Revolute }
            JointType::Continuous => { OJointType::Continuous }
            JointType::Prismatic => { OJointType::Prismatic }
            JointType::Fixed => { OJointType::Fixed }
            JointType::Floating => { OJointType::Floating }
            JointType::Planar => { OJointType::Planar }
            JointType::Spherical => { OJointType::Spherical }
        }
    }
    pub fn num_dofs(&self) -> usize {
        match self {
            OJointType::Revolute => {1}
            OJointType::Continuous => {1}
            OJointType::Prismatic => {1}
            OJointType::Fixed => {0}
            OJointType::Floating => {6}
            OJointType::Planar => {2}
            OJointType::Spherical => {3}
        }
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OMimic<T: AD> {
    joint: String,
    #[serde_as(as = "Option<SerdeAD<T>>")]
    multiplier: Option<T>,
    #[serde_as(as = "Option<SerdeAD<T>>")]
    offset: Option<T>
}
impl<T: AD> OMimic<T> {
    fn from_mimic(mimic: &Mimic) -> Self {
        Self {
            joint: String::from(&mimic.joint),
            multiplier: match &mimic.multiplier {
                None => { None }
                Some(multiplier) => { Some(T::constant(*multiplier)) }
            },
            offset: match &mimic.offset {
                None => { None }
                Some(offset) => { Some(T::constant(*offset)) }
            }
        }
    }
    pub fn joint(&self) -> &str {
        &self.joint
    }
    pub fn multiplier(&self) -> &Option<T> {
        &self.multiplier
    }
    pub fn offset(&self) -> &Option<T> {
        &self.offset
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ODynamics<T: AD> {
    #[serde_as(as = "SerdeAD<T>")]
    damping: T,
    #[serde_as(as = "SerdeAD<T>")]
    friction: T
}
impl<T: AD> ODynamics<T> {
    fn from_dynamics(dynamics: &Dynamics) -> Self {
        Self {
            damping: T::constant(dynamics.damping),
            friction: T::constant(dynamics.friction)
        }
    }
    pub fn damping(&self) -> &T {
        &self.damping
    }
    pub fn friction(&self) -> &T {
        &self.friction
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OJointLimit<T: AD> {
    #[serde_as(as = "SerdeAD<T>")]
    effort: T,
    #[serde_as(as = "SerdeAD<T>")]
    lower: T,
    #[serde_as(as = "SerdeAD<T>")]
    upper: T,
    #[serde_as(as = "SerdeAD<T>")]
    velocity: T
}
impl<T: AD> OJointLimit<T> {
    fn from_joint_limit(joint_limit: &JointLimit) -> Self {
        Self {
            effort:   T::constant(joint_limit.effort),
            lower:    T::constant(joint_limit.lower),
            upper:    T::constant(joint_limit.upper),
            velocity: T::constant(joint_limit.velocity)
        }
    }
    pub fn effort(&self) -> &T {
        &self.effort
    }
    pub fn lower(&self) -> &T {
        &self.lower
    }
    pub fn upper(&self) -> &T {
        &self.upper
    }
    pub fn velocity(&self) -> &T {
        &self.velocity
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OSafetyController<T: AD> {
    #[serde_as(as = "SerdeAD<T>")]
    k_position: T,
    #[serde_as(as = "SerdeAD<T>")]
    k_velocity: T,
    #[serde_as(as = "SerdeAD<T>")]
    soft_lower_limit: T,
    #[serde_as(as = "SerdeAD<T>")]
    soft_upper_limit: T
}
impl<T: AD> OSafetyController<T> {
    fn from_safety_controller(safety_controller: &SafetyController) -> Self {
        Self {
            k_position:       T::constant(safety_controller.k_position),
            k_velocity:       T::constant(safety_controller.k_velocity),
            soft_lower_limit: T::constant(safety_controller.soft_lower_limit),
            soft_upper_limit: T::constant(safety_controller.soft_upper_limit)
        }
    }
    pub fn k_position(&self) -> &T {
        &self.k_position
    }
    pub fn k_velocity(&self) -> &T {
        &self.k_velocity
    }
    pub fn soft_lower_limit(&self) -> &T {
        &self.soft_lower_limit
    }
    pub fn soft_upper_limit(&self) -> &T {
        &self.soft_upper_limit
    }
}