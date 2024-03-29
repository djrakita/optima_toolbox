use std::fmt::{Debug, Formatter};
use std::marker::PhantomData;
use serde::{Serialize, Deserialize};
use ad_trait::AD;
use urdf_rs::{Collision, Color, Dynamics, Geometry, Inertial, Joint, JointLimit, JointType, Link, Material, Mimic, Pose, SafetyController, Texture, Visual};
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategory};
use optima_3d_spatial::optima_3d_rotation::{O3DRotation};
use optima_3d_spatial::optima_3d_vec::O3DVec;
use optima_linalg::{OLinalgCategory, OMat};
use serde_with::*;
use crate::robotics_traits::{JointTrait};
use ad_trait::SerdeAD;
use optima_linalg::SerdeOMat;
use optima_3d_spatial::optima_3d_pose::SerdeO3DPose;
use optima_file::path::OStemCellPath;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ChainInfo {
    pub (crate) joint_parent_link: Vec<usize>,
    pub (crate) joint_child_link: Vec<usize>,
    pub (crate) link_parent_joint: Vec<Option<usize>>,
    pub (crate) link_children_joints: Vec<Vec<usize>>,
    pub (crate) link_parent_link: Vec<Option<usize>>,
    pub (crate) link_children_links: Vec<Vec<usize>>,
    pub (crate) base_link_idx: usize,
    pub (crate) kinematic_hierarchy: Vec<Vec<usize>>,
    pub (crate) link_connection_paths: Vec<Vec<Option<Vec<usize>>>>
}
impl ChainInfo {
    #[inline]
    pub fn joint_parent_link(&self, joint_idx: usize) -> usize {
        self.joint_parent_link[joint_idx]
    }
    #[inline]
    pub fn joint_child_link(&self, joint_idx: usize) -> usize { self.joint_child_link[joint_idx] }
    #[inline]
    pub fn link_parent_joint(&self, link_idx: usize) -> &Option<usize> { &self.link_parent_joint[link_idx] }
    #[inline]
    pub fn link_children_joints(&self, link_idx: usize) -> &Vec<usize> { &self.link_children_joints[link_idx] }
    #[inline]
    pub fn link_parent_link(&self, link_idx: usize) -> &Option<usize> { &self.link_parent_link[link_idx] }
    #[inline]
    pub fn link_children_links(&self, link_idx: usize) -> &Vec<usize> { &self.link_children_links[link_idx] }
    #[inline(always)]
    pub fn base_link_idx(&self) -> usize { self.base_link_idx }
    #[inline]
    pub fn kinematic_hierarchy(&self) -> &Vec<Vec<usize>> {
        &self.kinematic_hierarchy
    }
    #[inline]
    pub fn link_connection_path(&self, start_link: usize, end_link: usize) -> &Option<Vec<usize>> {
        &self.link_connection_paths[start_link][end_link]
    }
    #[inline]
    pub fn does_link_connection_path_exist(&self, start_link: usize, end_link: usize) -> bool {
        return self.link_connection_path(start_link, end_link).is_some()
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct OLink<T: AD, C: O3DPoseCategory, L: OLinalgCategory> {
    pub (crate) is_present_in_model: bool,
    pub (crate) link_idx: usize,
    pub (crate) sub_robot_idx: usize,
    pub (crate) link_idx_in_sub_robot: usize,
    pub (crate) name: String,
    pub (crate) parent_joint_idx: Option<usize>,
    pub (crate) children_joint_idxs: Vec<usize>,
    pub (crate) parent_link_idx: Option<usize>,
    pub (crate) children_link_idxs: Vec<usize>,
    pub (crate) link_connection_paths: Vec<Option<Vec<usize>>>,
    #[serde(deserialize_with = "Vec::<OCollision<T, C>>::deserialize")]
    collision: Vec<OCollision<T, C>>,
    #[serde(deserialize_with = "Vec::<OVisual<T, C>>::deserialize")]
    visual: Vec<OVisual<T, C>>,
    #[serde(deserialize_with = "OInertial::<T, L>::deserialize")]
    inertial: OInertial<T, L>,
    pub (crate) original_mesh_file_path: Option<OStemCellPath>,
    pub (crate) stl_mesh_file_path: Option<OStemCellPath>,
    pub (crate) convex_hull_file_path: Option<OStemCellPath>,
    pub (crate) convex_decomposition_file_paths: Vec<OStemCellPath>,
    pub (crate) convex_decomposition_levels_file_paths: Vec<Vec<OStemCellPath>>
}
impl<T: AD, C: O3DPoseCategory, L: OLinalgCategory> OLink<T, C, L> {
    pub (crate) fn from_link(link: &Link) -> Self {
        Self {
            is_present_in_model: true,
            link_idx: usize::default(),
            sub_robot_idx: 0,
            link_idx_in_sub_robot: usize::default(),
            name: String::from(&link.name),
            parent_joint_idx: None,
            children_joint_idxs: vec![],
            parent_link_idx: None,
            children_link_idxs: vec![],
            link_connection_paths: vec![],
            collision: link.collision.iter().map(|x| OCollision::from_collision(x)).collect(),
            visual: link.visual.iter().map(|x| OVisual::from_visual(x)).collect(),
            inertial: OInertial::from_inertial(&link.inertial),
            original_mesh_file_path: None,
            stl_mesh_file_path: None,
            convex_hull_file_path: None,
            convex_decomposition_file_paths: vec![],
            convex_decomposition_levels_file_paths: vec![],
        }
    }
    pub fn new_manual(name: &str, collision: Vec<OCollision<T, C>>, visual: Vec<OVisual<T, C>>, inertial: OInertial<T, L>) -> Self {
        Self {
            is_present_in_model: true,
            link_idx: usize::default(),
            sub_robot_idx: 0,
            link_idx_in_sub_robot: usize::default(),
            name: name.into(),
            parent_joint_idx: None,
            children_joint_idxs: vec![],
            parent_link_idx: None,
            children_link_idxs: vec![],
            link_connection_paths: vec![],
            collision,
            visual,
            inertial,
            original_mesh_file_path: None,
            stl_mesh_file_path: None,
            convex_hull_file_path: None,
            convex_decomposition_file_paths: vec![],
            convex_decomposition_levels_file_paths: vec![],
        }
    }
    #[inline(always)]
    pub fn is_present_in_model(&self) -> bool {
        self.is_present_in_model
    }
    #[inline(always)]
    pub fn link_idx(&self) -> usize {
        self.link_idx
    }
    #[inline(always)]
    pub fn sub_robot_idx(&self) -> usize {
        self.sub_robot_idx
    }
    #[inline(always)]
    pub fn link_idx_in_sub_chain(&self) -> usize {
        self.link_idx_in_sub_robot
    }
    #[inline(always)]
    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn collision(&self) -> &Vec<OCollision<T, C>> {
        &self.collision
    }
    pub fn visual(&self) -> &Vec<OVisual<T, C>> {
        &self.visual
    }
    pub fn inertial(&self) -> &OInertial<T, L> {
        &self.inertial
    }
    pub fn original_mesh_file_path(&self) -> &Option<OStemCellPath> {
        &self.original_mesh_file_path
    }
    pub fn stl_mesh_file_path(&self) -> &Option<OStemCellPath> {
        &self.stl_mesh_file_path
    }
    pub fn convex_hull_file_path(&self) -> &Option<OStemCellPath> {
        &self.convex_hull_file_path
    }
    pub fn convex_decomposition_file_paths(&self) -> &Vec<OStemCellPath> {
        &self.convex_decomposition_file_paths
    }
    pub fn convex_decomposition_levels_file_paths(&self) -> &Vec<Vec<OStemCellPath>> {
        &self.convex_decomposition_levels_file_paths
    }
}
impl<T: AD, C: O3DPoseCategory, L: OLinalgCategory> Debug for OLink<T, C, L> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let mut s = "".to_string();

        s += &format!("{{\n");
        s += &format!("  OLink\n");
        s += &format!("  Link name: {}\n", self.name());
        s += &format!("  Link idx: {}\n", self.link_idx());
        s += &format!("  Sub chain idx: {}\n", self.sub_robot_idx);
        s += &format!("  Link idx in sub chain: {}\n", self.link_idx_in_sub_robot);
        s += &format!("  Parent Link idx: {:?}\n", self.parent_link_idx);
        s += &format!("  Children Link idxs: {:?}\n", self.children_link_idxs);
        s += &format!("  Parent Joint idx: {:?}\n", self.parent_joint_idx);
        s += &format!("  Children Joint idxs: {:?}\n", self.children_joint_idxs);
        s += &format!("  Original mesh file path: {:?}\n", self.original_mesh_file_path);
        s += &format!("  Stl mesh file path: {:?}\n", self.stl_mesh_file_path);
        s += &format!("  Convex hull file path: {:?}\n", self.convex_hull_file_path);
        s += &format!("  Num convex subcomponents: {:?}\n", self.convex_decomposition_file_paths.len());
        s += &format!("}}");

        f.write_str(&s)?;
        Ok(())
    }
}

#[serde_as]
#[derive(Clone, Serialize, Deserialize)]
pub struct OJoint<T: AD, C: O3DPoseCategory> {
    pub (crate) is_present_in_model: bool,
    pub (crate) joint_idx: usize,
    pub (crate) name: String,
    pub (crate) joint_type: OJointType,
    #[serde_as(as = "Option::<Vec<SerdeAD<T>>>")]
    pub (crate) fixed_values: Option<Vec<T>>,
    #[serde(deserialize_with = "OPose::<T, C>::deserialize")]
    pub (crate) origin: OPose<T, C>,
    #[serde_as(as = "[SerdeAD<T>; 3]")]
    pub (crate) axis: [T; 3],
    pub (crate) parent_link: String,
    pub (crate) parent_link_idx: usize,
    pub (crate) child_link: String,
    pub (crate) child_link_idx: usize,
    pub (crate) dof_idxs: Vec<usize>,
    pub (crate) dof_idxs_range: Option<(usize, usize)>,
    #[serde(deserialize_with = "OJointLimit::<T>::deserialize")]
    pub (crate) limit: OJointLimit<T>,
    #[serde(deserialize_with = "Option::<ODynamics::<T>>::deserialize")]
    pub (crate) dynamics: Option<ODynamics<T>>,
    #[serde(deserialize_with = "Option::<OMimic<T>>::deserialize")]
    pub (crate) mimic: Option<OMimic<T>>,
    #[serde(deserialize_with = "Option::<OSafetyController<T>>::deserialize")]
    pub (crate) safety_controller: Option<OSafetyController<T>>
}
impl<T: AD, C: O3DPoseCategory> OJoint<T, C> {
    pub (crate) fn from_joint(joint: &Joint) -> Self {
        let axis: Vec<T> = joint.axis.xyz.0.iter().map(|x| T::constant(*x)).collect();

        Self {
            is_present_in_model: true,
            joint_idx: usize::default(),
            name: String::from(&joint.name),
            joint_type: OJointType::from_joint_type(&joint.joint_type),
            fixed_values: None,
            origin: OPose::from_pose(&joint.origin),
            axis: [axis[0], axis[1], axis[2]],
            parent_link: String::from(&joint.parent.link),
            parent_link_idx: usize::default(),
            child_link: String::from(&joint.child.link),
            child_link_idx: usize::default(),
            dof_idxs: vec![],
            dof_idxs_range: None,
            limit: OJointLimit::from_joint_limit(&joint.limit, &joint.joint_type),
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
    pub fn new_manual(name: &str, joint_type: OJointType, origin: C::P<T>, axis: [T; 3], parent_link: &str, child_link: &str, limit: OJointLimit<T>, dynamics: Option<ODynamics<T>>, mimic: Option<OMimic<T>>, safety_controller: Option<OSafetyController<T>>) -> Self {
        Self {
            is_present_in_model: true,
            joint_idx: usize::default(),
            name: name.into(),
            joint_type,
            fixed_values: None,
            origin: OPose::from_o3d_pose(&origin),
            axis,
            parent_link: parent_link.into(),
            parent_link_idx: usize::default(),
            child_link: child_link.into(),
            child_link_idx: usize::default(),
            dof_idxs: vec![],
            dof_idxs_range: None,
            limit,
            dynamics,
            mimic,
            safety_controller,
        }
    }
    #[inline(always)]
    pub fn is_present_in_model(&self) -> bool {
        self.is_present_in_model
    }
    #[inline(always)]
    pub fn name(&self) -> &str {
        &self.name
    }
    #[inline(always)]
    pub fn joint_type(&self) -> &OJointType {
        &self.joint_type
    }
    #[inline(always)]
    pub fn parent_link(&self) -> &str {
        &self.parent_link
    }
    #[inline(always)]
    pub fn parent_link_idx(&self) -> usize {
        self.parent_link_idx
    }
    #[inline(always)]
    pub fn child_link(&self) -> &str {
        &self.child_link
    }
    #[inline(always)]
    pub fn child_link_idx(&self) -> usize {
        self.child_link_idx
    }
    #[inline(always)]
    pub fn limit(&self) -> &OJointLimit<T> {
        &self.limit
    }
    #[inline(always)]
    pub fn dynamics(&self) -> &Option<ODynamics<T>> {
        &self.dynamics
    }
    #[inline(always)]
    pub fn mimic(&self) -> &Option<OMimic<T>> {
        &self.mimic
    }
    #[inline(always)]
    pub fn safety_controller(&self) -> &Option<OSafetyController<T>> {
        &self.safety_controller
    }
    #[inline(always)]
    pub fn fixed_values(&self) -> &Option<Vec<T>> {
        &self.fixed_values
    }
    #[inline(always)]
    pub fn origin(&self) -> &OPose<T, C> {
        &self.origin
    }
    #[inline(always)]
    pub fn axis(&self) -> &[T; 3] {
        &self.axis
    }
    #[inline(always)]
    pub fn joint_idx(&self) -> usize {
        self.joint_idx
    }
    #[inline(always)]
    pub fn dof_idxs(&self) -> &Vec<usize> {
        &self.dof_idxs
    }
    #[inline(always)]
    pub fn dof_idxs_range(&self) -> Option<(usize, usize)> {
        self.dof_idxs_range
    }
}
impl<T: AD, C: O3DPoseCategory + 'static> JointTrait<T, C> for OJoint<T, C> {
    #[inline(always)]
    fn name(&self) -> &str {
        &self.name
    }

    #[inline(always)]
    fn joint_idx(&self) -> usize {
        self.joint_idx
    }

    #[inline(always)]
    fn parent_link_idx(&self) -> usize {
        self.parent_link_idx
    }

    #[inline(always)]
    fn child_link_idx(&self) -> usize {
        self.child_link_idx
    }

    #[inline(always)]
    fn joint_type(&self) -> &OJointType {
        &self.joint_type
    }

    #[inline(always)]
    fn axis(&self) -> &[T; 3] {
        &self.axis
    }

    #[inline(always)]
    fn origin(&self) -> &OPose<T, C> {
        &self.origin
    }

    #[inline(always)]
    fn limit(&self) -> &OJointLimit<T> {
        &self.limit
    }

    #[inline(always)]
    fn mimic(&self) -> &Option<OMimic<T>> {
        &self.mimic
    }

    #[inline(always)]
    fn dynamics(&self) -> &Option<ODynamics<T>> {
        &self.dynamics
    }

    #[inline(always)]
    fn safety_controller(&self) -> &Option<OSafetyController<T>> {
        &self.safety_controller
    }

    #[inline(always)]
    fn get_num_dofs(&self) -> usize {
        return if !self.is_present_in_model { 0 } else if self.mimic.is_some() { 0 } else if self.fixed_values.is_some() { 0 } else { self.joint_type.num_dofs() }
    }

    #[inline(always)]
    fn fixed_values(&self) -> &Option<Vec<T>> {
        &self.fixed_values
    }

    #[inline(always)]
    fn dof_idxs(&self) -> &Vec<usize> {
        &self.dof_idxs
    }

    #[inline(always)]
    fn dof_idxs_range(&self) -> &Option<(usize, usize)> {
        &self.dof_idxs_range
    }
}
impl<T: AD, C: O3DPoseCategory + 'static> Debug for OJoint<T, C> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let mut s = "".to_string();

        s += &format!("{{\n");
        s += &format!("  OJoint\n");
        s += &format!("  Joint name: {}\n", self.name());
        s += &format!("  Joint idx: {}\n", self.joint_idx());
        s += &format!("  Present in model: {}\n", self.is_present_in_model());
        s += &format!("  Joint Type: {:?}\n", self.joint_type);
        s += &format!("  Axis: {:?}\n", self.axis);
        s += &format!("  DOF idxs: {:?}\n", self.dof_idxs);
        s += &format!("  Parent Link idx: {:?}\n", self.parent_link_idx);
        s += &format!("  Child Link idx: {:?}\n", self.child_link_idx);
        s += &format!("  Joint upper bounds: {:?}\n", self.limit().upper);
        s += &format!("  Joint lower bounds: {:?}\n", self.limit().lower);
        s += &format!("}}");

        f.write_str(&s)?;
        Ok(())
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ORobotSetJoint<T: AD, C: O3DPoseCategory> {
    macro_joint_idx: usize,
    #[serde(deserialize_with = "OPose::<T, C>::deserialize")]
    origin: OPose<T, C>,
    #[serde_as(as = "[SerdeAD<T>; 3]")]
    axis: [T; 3],
    joint_type: OJointType,
    #[serde(deserialize_with = "OJointLimit::<T>::deserialize")]
    limit: OJointLimit<T>,
    #[serde_as(as = "Option::<Vec<SerdeAD<T>>>")]
    pub (crate) fixed_values: Option<Vec<T>>,
    pub (crate) parent_chain_idx: usize,
    pub (crate) parent_link_idx_in_parent_chain: usize,
    pub (crate) child_chain_idx: usize,
    pub (crate) dof_idxs: Vec<usize>,
    pub (crate) dof_idxs_range: Option<(usize, usize)>,
    #[serde(deserialize_with = "Option::<OMimic<T>>::deserialize")]
    mimic: Option<OMimic<T>>,
    #[serde(deserialize_with = "Option::<ODynamics::<T>>::deserialize")]
    dynamics: Option<ODynamics<T>>,
    #[serde(deserialize_with = "Option::<OSafetyController<T>>::deserialize")]
    safety_controller: Option<OSafetyController<T>>,
    _phantom_data: PhantomData<T>
}
impl<T: AD, C: O3DPoseCategory> ORobotSetJoint<T, C> {
    pub (crate) fn new(macro_joint_idx: usize, origin: OPose<T, C>, axis: [T; 3], joint_type: OJointType, limit: OJointLimit<T>, parent_chain_idx: usize, parent_link_idx_in_parent_chain: usize, child_chain_idx: usize) -> Self {
        Self {
            macro_joint_idx: macro_joint_idx,
            origin,
            axis,
            joint_type,
            limit,
            fixed_values: None,
            parent_chain_idx,
            parent_link_idx_in_parent_chain,
            child_chain_idx,
            dof_idxs: Default::default(),
            dof_idxs_range: None,
            mimic: None,
            dynamics: None,
            safety_controller: None,
            _phantom_data: Default::default(),
        }
    }
    pub fn macro_joint_idx(&self) -> usize {
        self.macro_joint_idx
    }
    pub fn origin(&self) -> &OPose<T, C> {
        &self.origin
    }
    pub fn axis(&self) -> &[T; 3] {
        &self.axis
    }
    pub fn joint_type(&self) -> &OJointType {
        &self.joint_type
    }
    pub fn parent_chain_idx(&self) -> usize {
        self.parent_chain_idx
    }
    pub fn parent_link_idx_in_parent_chain(&self) -> usize {
        self.parent_link_idx_in_parent_chain
    }
    pub fn child_chain_idx(&self) -> usize {
        self.child_chain_idx
    }
    pub fn safety_controller(&self) -> &Option<OSafetyController<T>> {
        &self.safety_controller
    }
}
impl<T: AD, C: O3DPoseCategory + 'static> JointTrait<T, C> for ORobotSetJoint<T, C> {
    #[inline(always)]
    fn name(&self) -> &str {
        todo!()
    }

    #[inline(always)]
    fn joint_idx(&self) -> usize {
        self.macro_joint_idx
    }

    #[inline(always)]
    fn parent_link_idx(&self) -> usize {
        self.parent_chain_idx
    }

    #[inline(always)]
    fn child_link_idx(&self) -> usize {
        self.child_chain_idx
    }

    #[inline(always)]
    fn joint_type(&self) -> &OJointType {
        &self.joint_type
    }

    #[inline(always)]
    fn axis(&self) -> &[T; 3] {
        &self.axis
    }

    #[inline(always)]
    fn origin(&self) -> &OPose<T, C> {
        &self.origin
    }

    #[inline(always)]
    fn limit(&self) -> &OJointLimit<T> {
        &self.limit
    }

    #[inline(always)]
    fn mimic(&self) -> &Option<OMimic<T>> {
        &self.mimic
    }

    #[inline(always)]
    fn dynamics(&self) -> &Option<ODynamics<T>> {
        &self.dynamics
    }

    #[inline(always)]
    fn safety_controller(&self) -> &Option<OSafetyController<T>> {
        &self.safety_controller
    }

    #[inline(always)]
    fn get_num_dofs(&self) -> usize {
        self.joint_type.num_dofs()
    }

    #[inline(always)]
    fn fixed_values(&self) -> &Option<Vec<T>> {
        &self.fixed_values
    }

    #[inline(always)]
    fn dof_idxs(&self) -> &Vec<usize> {
        &self.dof_idxs
    }

    #[inline(always)]
    fn dof_idxs_range(&self) -> &Option<(usize, usize)> {
        &self.dof_idxs_range
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OInertial<T: AD, L: OLinalgCategory> {
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
impl<T: AD, L: OLinalgCategory> OInertial<T, L> {
    pub (crate) fn from_inertial(inertial: &Inertial) -> Self {
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
    pub fn new_manual(ixx: T, ixy: T, ixz: T, iyy: T, iyz: T, izz: T) -> Self {
        let mat_slice = [
            ixx,
            ixy,
            ixz,
            ixy,
            iyy,
            iyz,
            ixz,
            iyz,
            izz
        ];

        let inertial_matrix = L::MatType::from_column_major_slice(&mat_slice, 3, 3);

        Self {
            inertial_matrix,
            ixx,
            ixy,
            ixz,
            iyy,
            iyz,
            izz
        }
    }
    pub fn new_zeros() -> Self {
        Self::new_manual(T::zero(), T::zero(), T::zero(), T::zero(), T::zero(), T::zero())
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
pub struct OCollision<T: AD, C: O3DPoseCategory> {
    geometry: OGeometry,
    name: Option<String>,
    #[serde(deserialize_with = "OPose::<T, C>::deserialize")]
    origin: OPose<T, C>
}
impl<T: AD, C: O3DPoseCategory> OCollision<T, C> {
    pub (crate) fn from_collision(collision: &Collision) -> Self {
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
    pub fn origin(&self) -> &OPose<T, C> {
        &self.origin
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OVisual<T: AD, C: O3DPoseCategory> {
    name: Option<String>,
    material: Option<OMaterial>,
    #[serde(deserialize_with = "OPose::<T, C>::deserialize")]
    origin: OPose<T, C>,
    #[serde(deserialize_with = "OGeometry::deserialize")]
    geometry: OGeometry
}
impl<T: AD, C: O3DPoseCategory> OVisual<T, C> {
    pub (crate) fn from_visual(visual: &Visual) -> Self {
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
    pub fn name(&self) -> &Option<String> {
        &self.name
    }
    pub fn material(&self) -> &Option<OMaterial> {
        &self.material
    }
    pub fn origin(&self) -> &OPose<T, C> {
        &self.origin
    }
    pub fn geometry(&self) -> &OGeometry {
        &self.geometry
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
    pub (crate) fn from_material(material: &Material) -> Self {
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
    pub (crate) fn from_texture(texture: &Texture) -> Self {
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
    pub (crate) fn from_color(color: &Color) -> Self {
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
    pub (crate) fn from_geometry(geometry: &Geometry) -> Self {
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
pub struct OPose<T: AD, C: O3DPoseCategory> {
    #[serde_as(as = "SerdeO3DPose<T, C::P<T>>")]
    pub (crate) pose: C::P<T>,
    #[serde_as(as = "[SerdeAD<T>; 3]")]
    pub (crate) rpy: [T; 3],
    #[serde_as(as = "[SerdeAD<T>; 3]")]
    pub (crate) xyz: [T; 3]
}
impl<T: AD, C: O3DPoseCategory> OPose<T, C> {
    pub fn from_o3d_pose(pose: &C::P<T>) -> Self {
        let rpy = pose.rotation().euler_angles();
        let xyz = pose.translation().to_arr();

        Self {
            pose: pose.clone(),
            rpy,
            xyz,
        }
    }
    pub (crate) fn from_pose(pose: &Pose) -> Self {
        let rpy: Vec<T> = pose.rpy.0.iter().map(|x| T::constant(*x)).collect();
        let xyz: Vec<T> = pose.xyz.0.iter().map(|x| T::constant(*x)).collect();

        let pose = C::P::from_constructors(&xyz, &rpy);

        Self {
            pose,
            rpy: [rpy[0], rpy[1], rpy[2]],
            xyz: [xyz[0], xyz[1], xyz[2]]
        }
    }
    pub fn pose(&self) -> &C::P<T> {
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
    pub (crate) fn from_joint_type(joint_type: &JointType) -> Self {
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
    pub (crate) joint_idx: usize,
    pub (crate) joint: String,
    #[serde_as(as = "Option<SerdeAD<T>>")]
    multiplier: Option<T>,
    #[serde_as(as = "Option<SerdeAD<T>>")]
    offset: Option<T>
}
impl<T: AD> OMimic<T> {
    pub (crate) fn from_mimic(mimic: &Mimic) -> Self {
        Self {
            joint_idx: usize::default(),
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
    pub fn joint_idx(&self) -> usize {
        self.joint_idx
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
    pub (crate) fn from_dynamics(dynamics: &Dynamics) -> Self {
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

/// the Vec<...> is for all sub dofs (i.e., a floating joint will have six values for all fields)
#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OJointLimit<T: AD> {
    #[serde_as(as = "Vec<SerdeAD<T>>")]
    effort: Vec<T>,
    #[serde_as(as = "Vec<SerdeAD<T>>")]
    lower: Vec<T>,
    #[serde_as(as = "Vec<SerdeAD<T>>")]
    upper: Vec<T>,
    #[serde_as(as = "Vec<SerdeAD<T>>")]
    velocity: Vec<T>
}
impl<T: AD> OJointLimit<T> {
    pub (crate) fn from_joint_limit(joint_limit: &JointLimit, joint_type: &JointType) -> Self {
        let mut out = Self {
            effort:   vec![T::constant(joint_limit.effort)],
            lower:    vec![ T::constant(joint_limit.lower)],
            upper:    vec![ T::constant(joint_limit.upper)],
            velocity: vec![ T::constant(joint_limit.velocity)]
        };

        match joint_type {
            JointType::Continuous => {
                out.lower = vec![T::constant(-100.0)];
                out.upper = vec![T::constant(100.0)];
            }
            _ => { }
        }

        out
    }
    pub fn new_manual(effort: Vec<T>, lower: Vec<T>, upper: Vec<T>, velocity: Vec<T>) -> Self {
        Self {
            effort,
            lower,
            upper,
            velocity,
        }
    }
    #[inline]
    pub fn effort(&self) -> &Vec<T> {
        &self.effort
    }
    #[inline]
    pub fn lower(&self) -> &Vec<T> {
        &self.lower
    }
    #[inline]
    pub fn upper(&self) -> &Vec<T> {
        &self.upper
    }
    #[inline]
    pub fn velocity(&self) -> &Vec<T> {
        &self.velocity
    }
}
impl<T: AD> Default for OJointLimit<T> {
    fn default() -> Self {
        Self {
            effort: vec![T::zero()],
            lower: vec![T::zero()],
            upper: vec![T::zero()],
            velocity: vec![T::zero()],
        }
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
    pub (crate) fn from_safety_controller(safety_controller: &SafetyController) -> Self {
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

