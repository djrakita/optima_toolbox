use std::collections::HashMap;
use std::fmt::Debug;
use serde::{Serialize, Deserialize};
use std::marker::PhantomData;
use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryIsometry3, O3DPoseCategoryTrait};
use optima_linalg::{OLinalgCategoryNalgebra, OLinalgCategoryTrait, OVec};
use serde_with::*;
use optima_file::traits::{FromJsonString, ToJsonString};
use crate::robot::{ORobot};
use crate::robotics_components::*;
use crate::robotics_functions::compute_chain_info;
use crate::robotics_traits::{AsRobotTrait, JointTrait};
pub type ORobotSetDefault = ORobotSet<f64, O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra>;
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ORobotSet<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> {
    #[serde(deserialize_with = "Vec::<ORobotWrapper<T, C, L>>::deserialize")]
    robot_wrappers: Vec<ORobotWrapper<T, C, L>>,
    #[serde(deserialize_with = "Vec::<ORobotSetJoint<T, C>>::deserialize")]
    robot_set_joints: Vec<ORobotSetJoint<T, C>>,
    kinematic_hierarchy_of_robots: Vec<Vec<usize>>,
    num_dofs: usize,
    base_robot_idx: usize,
    dof_to_joint_and_sub_dof_idxs: Vec<RobotSetJointIdx>,
    #[serde(deserialize_with = "ORobot::<T, C, L>::deserialize")]
    robot_set_as_single_robot: ORobot<T, C, L>,
    _phantom_data: PhantomData<(T, C, L)>,
}
impl<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> ORobotSet<T, C, L> {
    pub fn new_empty() -> Self {
        Self {
            robot_wrappers: vec![ORobotWrapper::<T, C, L>::new_world_robot()],
            robot_set_joints: vec![],
            kinematic_hierarchy_of_robots: vec![],
            num_dofs: usize::default(),
            base_robot_idx: usize::default(),
            dof_to_joint_and_sub_dof_idxs: vec![],
            robot_set_as_single_robot: ORobot::new_empty(),
            _phantom_data: Default::default(),
        }
    }
    pub fn new_from_single_robot(robot: ORobot<T, C, L>) -> Self {
        let mut out = Self::new_empty();
        out.add_robot(robot, 0, 0, &C::P::<T>::identity(), [T::zero(); 3], OJointType::Fixed, OJointLimit::default());
        out
    }
    pub fn new_from_single_robot_name(robot_name: &str) -> Self {
        Self::new_from_single_robot(ORobot::from_urdf(robot_name))
    }
    pub fn to_new_generic_types<T2: AD, C2: O3DPoseCategoryTrait, L2: OLinalgCategoryTrait>(&self) -> ORobotSet<T2, C2, L2> {
        let json_str = self.to_json_string();
        ORobotSet::<T2, C2, L2>::from_json_string(&json_str)
    }
    pub fn to_new_ad_type<T2: AD>(&self) -> ORobotSet<T2, C, L> {
        self.to_new_generic_types::<T2, C, L>()
    }
    pub fn add_robot(&mut self, robot: ORobot<T, C, L>, parent_robot_idx: usize, parent_link_idx_in_parent_robot: usize, origin: &C::P<T>, axis: [T; 3], joint_type: OJointType, limit: OJointLimit<T>) {
        assert!(parent_robot_idx <= self.robot_wrappers.len());
        assert!(parent_link_idx_in_parent_robot <= self.robot_wrappers[parent_robot_idx].robot.joints().len());
        assert!(limit.upper().len() >= joint_type.num_dofs());
        assert!(limit.lower().len() >= joint_type.num_dofs());
        assert!(limit.velocity().len() >= joint_type.num_dofs());
        assert!(limit.effort().len() >= joint_type.num_dofs());

        let new_robot_idx = self.robot_wrappers.len();
        let new_macro_joint_idx = self.robot_set_joints.len();
        let pose = OPose::from_o3d_pose(origin);
        let macro_joint = ORobotSetJoint::new(new_macro_joint_idx, pose, axis, joint_type, limit, parent_robot_idx, parent_link_idx_in_parent_robot, new_robot_idx);

        let robot_wrapper = ORobotWrapper {
            robot,
            robot_idx: new_robot_idx,
            parent_robot_set_joint_idx: None,
            children_robot_set_joint_idxs: vec![],
            parent_robot_idx: None,
            parent_link_idx_in_parent_robot: None,
            children_robot_idxs: vec![],
            robot_connection_paths: vec![],
            dof_idxs: vec![],
            dof_idxs_range: None,
        };

        self.robot_wrappers.push(robot_wrapper);
        self.robot_set_joints.push(macro_joint);

        self.setup();
    }
    #[inline(always)]
    pub fn robot_wrappers(&self) -> &Vec<ORobotWrapper<T, C, L>> {
        &self.robot_wrappers
    }
    #[inline(always)]
    pub fn robot_set_joints(&self) -> &Vec<ORobotSetJoint<T, C>> {
        &self.robot_set_joints
    }
    #[inline(always)]
    pub fn kinematic_hierarchy_of_robots(&self) -> &Vec<Vec<usize>> {
        &self.kinematic_hierarchy_of_robots
    }
    #[inline(always)]
    pub fn num_dofs(&self) -> usize {
        self.num_dofs
    }
    #[inline(always)]
    pub fn base_robot_idx(&self) -> usize {
        self.base_robot_idx
    }
    #[inline]
    pub fn get_macro_joint_transform<V: OVec<T>>(&self, state: &V, macro_joint_idx: usize) -> C::P<T> {
        self.robot_set_joints[macro_joint_idx].get_joint_transform(state, &self.robot_set_joints)
    }
    #[inline(always)]
    pub fn get_macro_joint_fixed_offset_transform(&self, macro_joint_idx: usize) -> &C::P<T> {
        self.robot_set_joints[macro_joint_idx].get_joint_fixed_offset_transform()
    }
    #[inline]
    pub fn get_macro_joint_variable_transform<V: OVec<T>>(&self, state: &V, macro_joint_idx: usize) -> C::P<T> {
        self.robot_set_joints[macro_joint_idx].get_joint_variable_transform(state, &self.robot_set_joints)
    }
    #[inline]
    pub fn dof_to_joint_and_sub_dof_idxs(&self) -> &Vec<RobotSetJointIdx> {
        &self.dof_to_joint_and_sub_dof_idxs
    }
    pub (crate) fn get_robot_set_as_single_robot(&self) -> ORobot<T, C, L> {
        let mut links = vec![];
        let mut joints = vec![];

        let mut robot_and_link_idx_to_new_link_name_mapping = HashMap::new();

        self.robot_wrappers.iter().enumerate().for_each(|(robot_idx, robot_wrapper)| {
            robot_wrapper.robot.links().iter().enumerate().for_each(|(link_idx_in_robot, link)| {
                let link_name = format!("robot_{}_{}", robot_idx, link.name());
                robot_and_link_idx_to_new_link_name_mapping.insert((robot_idx as usize, link_idx_in_robot as usize), link_name.clone());
                let mut new_link = link.clone();
                new_link.name = link_name.clone();
                new_link.sub_robot_idx = robot_idx;
                new_link.link_idx_in_sub_robot = link_idx_in_robot;
                links.push(new_link);
            });
        });

        self.kinematic_hierarchy_of_robots.iter().for_each(|robot_idx_layer| {
            robot_idx_layer.iter().for_each(|robot_idx| {
                let robot_wrapper = self.robot_wrappers.get(*robot_idx).expect("error");
                let robot = robot_wrapper.robot();
                let robot_base_link_idx = robot.base_link_idx();

                let parent_robot_idx = robot_wrapper.parent_robot_idx;
                let parent_link_idx_in_parent_robot = robot_wrapper.parent_link_idx_in_parent_robot;
                let parent_macro_joint_idx = robot_wrapper.parent_robot_set_joint_idx;

                if let (Some(parent_robot_idx), Some(parent_link_idx_in_parent_robot), Some(parent_macro_joint_idx)) = (parent_robot_idx, parent_link_idx_in_parent_robot, parent_macro_joint_idx) {
                    let parent_link_name = robot_and_link_idx_to_new_link_name_mapping.get(&(parent_robot_idx, parent_link_idx_in_parent_robot)).expect("error");
                    let child_link_name = robot_and_link_idx_to_new_link_name_mapping.get(&(*robot_idx, robot_base_link_idx)).expect("error");
                    let joint = &self.robot_set_joints[parent_macro_joint_idx];

                    let new_joint = OJoint::new_manual(&format!("macro_joint_{}", parent_macro_joint_idx), joint.joint_type().clone(), joint.origin().pose.clone(), joint.axis().clone(), parent_link_name, child_link_name, joint.limit().clone(), joint.dynamics().clone(), joint.mimic().clone(), joint.safety_controller().clone());
                    joints.push(new_joint);
                }

                robot.joints().iter().for_each(|joint| {
                    let parent_link_name = robot_and_link_idx_to_new_link_name_mapping.get( &(*robot_idx, joint.parent_link_idx) ).expect("error");
                    let child_link_name = robot_and_link_idx_to_new_link_name_mapping.get( &(*robot_idx, joint.child_link_idx) ).expect("error");

                    let joint_name = format!("robot_{}_{}", robot_idx, joint.name());

                    let mut new_joint = joint.clone();
                    new_joint.name = joint_name;
                    new_joint.parent_link = parent_link_name.clone();
                    new_joint.child_link = child_link_name.clone();

                    joints.push(new_joint);
                });
            });
        });

        let mut out = ORobot::from_manual_no_mesh_setup("robot_set_as_robot", links, joints);

        self.robot_wrappers.iter().for_each(|x| {
            out.sub_robots.push(x.robot.clone());
        });

        out
    }
    fn setup(&mut self) {
        self.set_chain_info();
        self.set_num_dofs();
        self.set_all_sub_dof_idxs();
        self.set_dof_to_joint_and_sub_dof_idxs();
        self.set_robot_set_as_single_robot();
    }
}
impl<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait> ORobotSet<T, C, L> {
    fn set_chain_info(&mut self) {
        let chain_info = compute_chain_info(&self.robot_wrappers, &self.robot_set_joints);

        self.base_robot_idx = chain_info.base_link_idx;
        self.kinematic_hierarchy_of_robots = chain_info.kinematic_hierarchy.clone();

        self.robot_wrappers.iter_mut().enumerate().for_each(|(i, x)| {
            x.parent_robot_set_joint_idx = chain_info.link_parent_joint[i].clone();
            if let Some(link_parent_joint_idx) = &chain_info.link_parent_joint[i] {
                let macro_joint = self.robot_set_joints[*link_parent_joint_idx].clone();
                x.parent_link_idx_in_parent_robot = Some(macro_joint.parent_link_idx_in_parent_chain);
            }
            x.children_robot_set_joint_idxs = chain_info.link_children_joints[i].clone();
            x.parent_robot_idx = chain_info.link_parent_link[i].clone();
            x.children_robot_idxs = chain_info.link_children_links[i].clone();
            x.robot_connection_paths = chain_info.link_connection_paths[i].clone();
        });
    }
    fn set_num_dofs(&mut self) {
        let mut num_dofs = 0;
        self.robot_wrappers.iter().for_each(|x| num_dofs += x.robot.num_dofs());
        self.robot_set_joints.iter().for_each(|x| num_dofs += x.get_num_dofs());

        self.num_dofs = num_dofs;
    }
    fn set_all_sub_dof_idxs(&mut self) {
        let mut count = 0;

        let mut macro_joint_sub_dof_idxs = vec![vec![]; self.robot_set_joints.len()];
        let mut macro_joint_sub_dof_idxs_range = vec![None; self.robot_set_joints.len()];
        let mut robot_sub_dof_idxs = vec![vec![]; self.robot_wrappers.len()];
        let mut robot_sub_dof_idxs_range = vec![None; self.robot_wrappers.len()];

        let kinematic_hierarchy = &self.kinematic_hierarchy_of_robots;
        for layer in kinematic_hierarchy {
            for robot_idx in layer {
                let parent_joint = self.robot_wrappers[*robot_idx].parent_robot_set_joint_idx;
                match parent_joint {
                    None => {}
                    Some(parent_joint) => {
                        let num_dofs = self.robot_set_joints[parent_joint].get_num_dofs();
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

                let num_dofs = self.robot_wrappers[*robot_idx].robot.num_dofs();
                if num_dofs > 0 {
                    let mut sub_dof_idxs = vec![];
                    for _ in 0..num_dofs {
                        sub_dof_idxs.push(count);
                        count += 1;
                    }
                    robot_sub_dof_idxs_range[*robot_idx] = Some((*sub_dof_idxs.first().unwrap(), *sub_dof_idxs.last().unwrap() + 1));
                    robot_sub_dof_idxs[*robot_idx] = sub_dof_idxs;
                }
            }
        }

        self.robot_wrappers.iter_mut().enumerate().for_each(|(i, x)| {
            x.dof_idxs = robot_sub_dof_idxs[i].clone();
            x.dof_idxs_range = robot_sub_dof_idxs_range[i].clone();
        });

        self.robot_set_joints.iter_mut().enumerate().for_each(|(i, x)| {
            x.dof_idxs = macro_joint_sub_dof_idxs[i].clone();
            x.dof_idxs_range = macro_joint_sub_dof_idxs_range[i].clone();
        });
    }
    fn set_dof_to_joint_and_sub_dof_idxs(&mut self) {
        let mut dof_to_joint_and_sub_dof_idxs = vec![RobotSetJointIdx::default(); self.num_dofs];

        self.robot_set_joints.iter().enumerate().for_each(|(macro_joint_idx, macro_joint)| {
            macro_joint.dof_idxs.iter().enumerate().for_each(|(i, dof_idx)| {
                dof_to_joint_and_sub_dof_idxs[*dof_idx] = RobotSetJointIdx::RobotSetJoint { robot_set_joint_idx: macro_joint_idx, sub_dof_idx: i };
            });
        });

        self.robot_wrappers.iter().enumerate().for_each(|(robot_idx, robot_wrapper)| {
            robot_wrapper.dof_idxs.iter().enumerate().for_each(|(i, dof_idx)| {
                let (joint_idx, sub_dof_idx) = robot_wrapper.robot.dof_to_joint_and_sub_dof_idxs()[i].clone();

                dof_to_joint_and_sub_dof_idxs[*dof_idx] = RobotSetJointIdx::RobotJoint {
                    robot_idx,
                    joint_idx,
                    sub_dof_idx,
                };
            });
        });

        self.dof_to_joint_and_sub_dof_idxs = dof_to_joint_and_sub_dof_idxs;
    }
    fn set_robot_set_as_single_robot(&mut self) {
        self.robot_set_as_single_robot = self.get_robot_set_as_single_robot();
    }
}
impl<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> AsRobotTrait<T, C, L> for ORobotSet<T, C, L> {
    #[inline(always)]
    fn as_robot(&self) -> &ORobot<T, C, L> {
        &self.robot_set_as_single_robot
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ORobotWrapper<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait> {
    #[serde(deserialize_with = "ORobot::<T, C, L>::deserialize")]
    robot: ORobot<T, C, L>,
    robot_idx: usize,
    pub(crate) parent_robot_set_joint_idx: Option<usize>,
    pub(crate) children_robot_set_joint_idxs: Vec<usize>,
    pub(crate) parent_robot_idx: Option<usize>,
    pub(crate) parent_link_idx_in_parent_robot: Option<usize>,
    pub(crate) children_robot_idxs: Vec<usize>,
    pub(crate) robot_connection_paths: Vec<Option<Vec<usize>>>,
    dof_idxs: Vec<usize>,
    dof_idxs_range: Option<(usize, usize)>,
}
impl<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait> ORobotWrapper<T, C, L> {
    fn new_world_robot() -> Self {
        Self {
            robot: ORobot::new_world_robot(),
            robot_idx: 0,
            parent_robot_set_joint_idx: None,
            children_robot_set_joint_idxs: vec![],
            parent_robot_idx: None,
            parent_link_idx_in_parent_robot: None,
            children_robot_idxs: vec![],
            robot_connection_paths: vec![],
            dof_idxs: vec![],
            dof_idxs_range: None,
        }
    }
    #[inline(always)]
    pub fn robot(&self) -> &ORobot<T, C, L> {
        &self.robot
    }
    #[inline(always)]
    pub fn robot_idx(&self) -> usize {
        self.robot_idx
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
    pub fn robot_connection_paths(&self) -> &Vec<Option<Vec<usize>>> {
        &self.robot_connection_paths
    }
    #[inline(always)]
    pub fn parent_robot_set_joint_idx(&self) -> &Option<usize> {
        &self.parent_robot_set_joint_idx
    }
    #[inline(always)]
    pub fn children_robot_set_joint_idxs(&self) -> &Vec<usize> {
        &self.children_robot_set_joint_idxs
    }
    #[inline(always)]
    pub fn parent_robot_idx(&self) -> &Option<usize> {
        &self.parent_robot_idx
    }
    #[inline(always)]
    pub fn children_robot_idxs(&self) -> &Vec<usize> {
        &self.children_robot_idxs
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
impl<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait> ORobot<T, C, L> {
    pub(crate) fn new_world_robot() -> Self {
        Self::from_manual("world", vec![OLink::new_manual("world_link", vec![], vec![], OInertial::new_zeros())], vec![])
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum RobotSetJointIdx {
    RobotJoint { robot_idx: usize, joint_idx: usize, sub_dof_idx: usize },
    RobotSetJoint { robot_set_joint_idx: usize, sub_dof_idx: usize },
}
impl Default for RobotSetJointIdx {
    fn default() -> Self {
        Self::RobotJoint {
            robot_idx: 0,
            joint_idx: 0,
            sub_dof_idx: 0,
        }
    }
}