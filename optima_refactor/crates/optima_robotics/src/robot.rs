use std::borrow::Cow;
use std::cell::RefCell;
use std::collections::HashMap;
use std::fmt::{Debug, Formatter};
use std::marker::PhantomData;
use std::rc::Rc;
use ad_trait::*;
use ad_trait::differentiable_block::DifferentiableBlock;
use ad_trait::differentiable_function::DerivativeMethodTrait;
use serde::{Serialize, Deserialize};
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryIsometry3, O3DPoseCategory};
use crate::utils::get_urdf_path_from_chain_name;
use serde_with::*;
use optima_3d_mesh::{SaveToSTL, ToTriMesh};
use optima_3d_spatial::optima_3d_rotation::O3DRotation;
use optima_3d_spatial::optima_3d_vec::{O3DVec, O3DVecCategoryArr};
use optima_console::output::{oprint, PrintColor, PrintMode};
use optima_console::tab;
use optima_file::path::{OAssetLocation, OPath, OPathMatchingPattern, OPathMatchingStopCondition, OStemCellPath};
use optima_file::traits::{FromJsonString, ToJsonString};
use optima_linalg::{OLinalgCategoryNalgebra, OLinalgCategory, OVec, OVecCategoryVec};
use crate::robotics_components::*;
use crate::robotics_functions::compute_chain_info;
use crate::robotics_traits::{AsRobotTrait, JointTrait};
use optima_misc::arr_storage::MutArrTraitRaw;
use optima_misc::arr_storage::ImmutArrTraitRaw;
use optima_proximity::pair_group_queries::{OPairGroupQryTrait, OwnedPairGroupQry, PairGroupQryOutputCategoryParryFilter, PairGroupQryOutputCategory, ParryFilterOutput, ParryPairSelector, ToParryProximityOutputCategory, SkipReason};
use optima_proximity::shape_scene::ShapeSceneTrait;
use optima_proximity::shapes::{OParryShape, ShapeCategoryOParryShape};
use optima_sampling::SimpleSampler;
use optima_universal_hashmap::AHashMapWrapper;
use crate::robot_shape_scene::{ORobotParryShapeScene};
use crate::robotics_optimization::robotics_optimization_functions::{AxisDirection, LookAtTarget};
use crate::robotics_optimization::robotics_optimization_ik::{DifferentiableBlockIKObjective, DifferentiableFunctionClassIKObjective, DifferentiableFunctionIKObjective, IKGoal, IKGoalVecTrait};
use crate::robotics_optimization::robotics_optimization_look_at::{DifferentiableFunctionClassLookAt, DifferentiableFunctionLookAt};

pub type ORobotDefault = ORobot<f64, O3DPoseCategoryIsometry3, OLinalgCategoryNalgebra>;
#[serde_as]
#[derive(Clone, Serialize, Deserialize)]
pub struct ORobot<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory> {
    pub (crate) robot_name: String,
    robot_type: RobotType,
    #[serde(deserialize_with = "Vec::<OLink<T, C, L>>::deserialize")]
    links: Vec<OLink<T, C, L>>,
    #[serde(deserialize_with = "Vec::<OJoint<T, C>>::deserialize")]
    pub (crate) joints: Vec<OJoint<T, C>>,
    link_name_to_link_idx_map: HashMap<String, usize>,
    joint_name_to_joint_idx_map: HashMap<String, usize>,
    base_link_idx: usize,
    kinematic_hierarchy: Vec<Vec<usize>>,
    num_dofs: usize,
    dof_to_joint_and_sub_dof_idxs: Vec<(usize, usize)>,
    #[serde_as(as = "Vec<Vec<SerdeAD<T>>>")]
    non_collision_states: Vec<Vec<T>>,
    #[serde(deserialize_with = "Vec::<ORobot<T, C, L>>::deserialize")]
    pub (crate) sub_robots: Vec<ORobot<T, C, L>>,
    #[serde(deserialize_with = "ORobotParryShapeScene::<T, C, L>::deserialize")]
    pub (crate) parry_shape_scene: ORobotParryShapeScene<T, C, L>,
    has_been_preprocessed: bool,
    phantom_data: PhantomData<(T, C)>
}
impl<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static> ORobot<T, C, L> {
    pub fn from_urdf(robot_name: &str) -> Self {
        let urdf_path = get_urdf_path_from_chain_name(robot_name);
        let urdf = urdf_path.load_urdf();

        let mut links = vec![];
        let mut joints = vec![];

        urdf.links.iter().for_each(|x| {
            links.push(OLink::from_link(x));
        });

        urdf.joints.iter().for_each(|x| {
            joints.push(OJoint::from_joint(x));
        });

        Self::from_manual(robot_name, links, joints)
    }
    pub fn from_manual(robot_name: &str, links: Vec<OLink<T, C, L>>, joints: Vec<OJoint<T, C>>) -> Self {
        let mut link_name_to_link_idx_map = HashMap::new();
        let mut joint_name_to_joint_idx_map = HashMap::new();

        links.iter().enumerate().for_each(|(i, x)| {
            link_name_to_link_idx_map.insert( x.name().to_string(), i );
        });

        joints.iter().enumerate().for_each(|(i, x)| {
            joint_name_to_joint_idx_map.insert( x.name().to_string(), i );
        });

        let mut out = Self {
            robot_name: robot_name.into(),
            robot_type: RobotType::Robot,
            links,
            joints,
            link_name_to_link_idx_map,
            joint_name_to_joint_idx_map,
            base_link_idx: usize::default(),
            kinematic_hierarchy: vec![],
            num_dofs: usize::default(),
            dof_to_joint_and_sub_dof_idxs: vec![],
            non_collision_states: vec![],
            sub_robots: vec![],
            parry_shape_scene: ORobotParryShapeScene::new_default(),
            has_been_preprocessed: false,
            phantom_data: Default::default(),
        };

        out.setup();

        out
    }
    pub fn load_from_saved_robot(robot_name: &str) -> Self {
        let mut p = OStemCellPath::new_asset_path();
        p.append_file_location(&OAssetLocation::SavedRobot { robot_name });
        p.load_object_from_json_file::<ORobot<T, C, L>>()
    }
    pub fn save_robot(&mut self, name: Option<&str>) {
        if !self.has_been_preprocessed {
            oprint("cannot save a non-preprocessed robot.  returning.", PrintMode::Println, PrintColor::Yellow);
        }
        let name = match name {
            None => {
                match &self.robot_type {
                    RobotType::Robot => { self.robot_name.clone() }
                    RobotType::RobotSet => { self.robot_name.clone() }
                }
            }
            Some(name) => { name.to_string() }
        };
        if name == "robot_set_default" { panic!("cannot save robot with name robot_set_default"); }

        self.robot_name = name.clone();
        let mut p = OStemCellPath::new_asset_path();
        p.append_file_location(&OAssetLocation::SavedRobot { robot_name: &name });
        p.save_object_to_file_as_json(self);
    }
    pub (crate) fn from_manual_internal(robot_name: &str, links: Vec<OLink<T, C, L>>, joints: Vec<OJoint<T, C>>, robot_type: RobotType) -> Self {
        let mut link_name_to_link_idx_map = HashMap::new();
        let mut joint_name_to_joint_idx_map = HashMap::new();

        links.iter().enumerate().for_each(|(i, x)| {
            link_name_to_link_idx_map.insert( x.name().to_string(), i );
        });

        joints.iter().enumerate().for_each(|(i, x)| {
            joint_name_to_joint_idx_map.insert( x.name().to_string(), i );
        });

        let mut out = Self {
            robot_name: robot_name.into(),
            robot_type,
            links,
            joints,
            link_name_to_link_idx_map,
            joint_name_to_joint_idx_map,
            base_link_idx: usize::default(),
            kinematic_hierarchy: vec![],
            num_dofs: usize::default(),
            dof_to_joint_and_sub_dof_idxs: vec![],
            non_collision_states: vec![],
            sub_robots: vec![],
            parry_shape_scene: ORobotParryShapeScene::new_default(),
            has_been_preprocessed: false,
            phantom_data: Default::default(),
        };

        out.set_link_and_joint_idxs();
        out.assign_joint_connection_indices();
        out.set_mimic_joint_idxs();
        out.set_chain_info();
        out.set_num_dofs();
        out.set_all_sub_dof_idxs();
        out.set_dof_to_joint_and_sub_dof_idxs();
        out
    }
    pub (crate) fn new_empty() -> Self {
        Self {
            robot_name: "".to_string(),
            robot_type: RobotType::Robot,
            links: vec![],
            joints: vec![],
            link_name_to_link_idx_map: Default::default(),
            joint_name_to_joint_idx_map: Default::default(),
            base_link_idx: 0,
            kinematic_hierarchy: vec![],
            num_dofs: 0,
            dof_to_joint_and_sub_dof_idxs: vec![],
            non_collision_states: vec![],
            sub_robots: vec![],
            parry_shape_scene: ORobotParryShapeScene::new_default(),
            has_been_preprocessed: false,
            phantom_data: Default::default(),
        }
    }
    pub fn to_new_generic_types<T2: AD, C2: O3DPoseCategory, L2: OLinalgCategory>(&self) -> ORobot<T2, C2, L2> {
        let json_str = self.to_json_string();
        ORobot::<T2, C2, L2>::from_json_string(&json_str)
    }
    pub fn to_new_ad_type<T2: AD>(&self) -> ORobot<T2, C, L> {
        self.to_new_generic_types::<T2, C, L>()
    }
    #[inline(always)]
    pub fn robot_name(&self) -> &str {
        &self.robot_name
    }
    #[inline(always)]
    pub fn robot_type(&self) -> &RobotType {
        &self.robot_type
    }
    #[inline(always)]
    pub fn sub_robots(&self) -> &Vec<ORobot<T, C, L>> {
        &self.sub_robots
    }
    #[inline(always)]
    pub fn get_link_idx_from_link_name(&self, link_name: &str) -> usize {
        *self.link_name_to_link_idx_map.get(link_name).expect(&format!("link name {} not found", link_name))
    }
    #[inline(always)]
    pub fn get_joint_idx_from_joint_name(&self, joint_name: &str) -> usize {
        *self.joint_name_to_joint_idx_map.get(joint_name).expect(&format!("tried to find joint {}, could not find it", joint_name))
    }
    #[inline(always)]
    pub fn num_dofs(&self) -> usize {
        self.num_dofs
    }
    #[inline(always)]
    pub fn links(&self) -> &Vec<OLink<T, C, L>> {
        &self.links
    }
    #[inline(always)]
    pub fn joints(&self) -> &Vec<OJoint<T, C>> {
        &self.joints
    }
    pub fn print_joints(&self) {
        self.joints.iter().for_each(|x| {
           println!("{:?}", x);
        });
    }
    pub fn set_joint_as_fixed(&mut self, joint_idx: usize, fixed_values: &[T]) {
        let joint = self.joints.get_element_mut(joint_idx);

        if joint.mimic().is_some() { panic!("cannot fix joint values on a mimic joint.") }

        assert_eq!(fixed_values.len(), joint.joint_type().num_dofs(), "{}", format!("fixed values length {:?} must equal number of degrees of freedom of joint {:?}", fixed_values.len(), joint.joint_type().num_dofs()));

        joint.fixed_values = Some(fixed_values.into());

        self.joints.iter_mut().for_each(|x| {
           if let Some(mimic) = &x.mimic {
               if mimic.joint_idx == joint_idx { x.fixed_values = Some(fixed_values.into()); }
           }
        });

        self.set_num_dofs();
        self.set_all_sub_dof_idxs();
    }
    pub fn set_dead_end_link(&mut self, link_idx: usize) {
        self.links[link_idx].is_present_in_model = false;

        let mut link_idx_stack = vec![ link_idx ];

        while !link_idx_stack.is_empty() {
            let link_idx = link_idx_stack.pop().unwrap();
            self.links.get_element_mut(link_idx).is_present_in_model = false;
            // let parent_joint_idx = self.links.get_element(link_idx).parent_joint_idx();
            let parent_joint_idx = self.links.get(link_idx).unwrap().parent_joint_idx;
            if let Some(parent_joint_idx) = parent_joint_idx {
                self.joints[parent_joint_idx].is_present_in_model = false;
            }
            self.links.get(link_idx).as_ref().unwrap().children_link_idxs.iter().for_each(|x| link_idx_stack.push(*x));
        }

        self.set_num_dofs();
        self.set_all_sub_dof_idxs();
    }
    #[inline]
    pub fn get_joint_transform<V: OVec<T>>(&self, state: &V, joint_idx: usize) -> C::P<T> {
        self.joints[joint_idx].get_joint_transform(state, &self.joints)
    }
    #[inline(always)]
    pub fn get_joint_fixed_offset_transform(&self, joint_idx: usize) -> &C::P<T> {
        self.joints[joint_idx].get_joint_fixed_offset_transform()
    }
    #[inline]
    pub fn get_joint_variable_transform<V: OVec<T>>(&self, state: &V, joint_idx: usize) -> C::P<T> {
        self.joints[joint_idx].get_joint_variable_transform(state, &self.joints)
    }
    #[inline]
    pub fn dof_to_joint_and_sub_dof_idxs(&self) -> &Vec<(usize, usize)> {
        &self.dof_to_joint_and_sub_dof_idxs
    }
    #[inline]
    pub fn base_link_idx(&self) -> usize {
        self.base_link_idx
    }
    pub fn forward_kinematics<V: OVec<T>>(&self, state: &V, base_offset: Option<&C::P<T>>) -> FKResult<T, C::P<T>> {
        let mut out = vec![ None; self.links.len() ];

        let base_pose = match base_offset {
            None => { C::P::<T>::identity() }
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

        FKResult { link_poses: out, _phantom_data: Default::default() }
    }
    pub fn forward_kinematics_floating_chain<V: OVec<T>>(&self, state: &V, start_link_idx: usize, end_link_idx: usize, base_offset: Option<&C::P<T>>) -> FKResult<T, C::P<T>> {
        let mut out = vec![ None; self.links.len() ];

        let base_pose = match base_offset {
            None => { C::P::<T>::identity() }
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

        FKResult { link_poses: out, _phantom_data: Default::default() }
    }
    pub fn get_links_string(&self) -> String {
        let mut s = "".to_string();
        let mut it = self.links.iter().peekable();
        while let Some(link) = it.next() {
            s += &format!("{:?}", link);
            if it.peek().is_some() {
                s += "\n";
            }
        };
        s
    }
    pub fn get_joints_string(&self) -> String {
        let mut s = "".to_string();
        let mut it = self.joints.iter().peekable();
        while let Some(joint) = it.next() {
            s += &format!("{:?}", joint);
            if it.peek().is_some() {
                s += "\n";
            }
        };
        s
    }
    /*
    pub fn get_robot_parry_shape_scene(&self) -> ORobotParryShapeScene<T, C, L> {
        let mut shapes = vec![];
        let mut id_to_string = AHashMap::new();

        self.links.iter().for_each(|link| {
           if link.is_present_in_model {
               if let Some(convex_hull_file_path) = &link.convex_hull_file_path {
                   let convex_shape_trimesh = convex_hull_file_path.load_stl().to_trimesh();
                   let mut convex_shape_subcomponents_trimesh = vec![];
                   link.convex_decomposition_file_paths.iter().for_each(|x| {
                       convex_shape_subcomponents_trimesh.push(x.load_stl().to_trimesh());
                   });

                   let shape = OParryShape::new_convex_shape_from_trimesh(convex_shape_trimesh, C::P::identity(), Some(convex_shape_subcomponents_trimesh));

                   id_to_string.insert(shape.base_shape().base_shape().id(), format!("convex shape for link {} ({})", link.link_idx, link.name));
                   id_to_string.insert(shape.base_shape().obb().id(), format!("obb for link {} ({})", link.link_idx, link.name));
                   id_to_string.insert(shape.base_shape().bounding_sphere().id(), format!("bounding sphere for link {} ({})", link.link_idx, link.name));
                   shape.convex_subcomponents().iter().enumerate().for_each(|(i, x)| {
                       id_to_string.insert(x.base_shape().id(), format!("convex shape for link {} ({}) subcomponent {}", link.link_idx, link.name, i));
                       id_to_string.insert(x.obb().id(), format!("obb for link {} ({}) subcomponent {}", link.link_idx, link.name, i));
                       id_to_string.insert(x.bounding_sphere().id(), format!("bounding sphere for link {} ({}) subcomponent {}", link.link_idx, link.name, i));
                   });

                   shapes.push(shape);
               }
           }
        });

        ORobotParryShapeScene {
            shapes,
            pair_skips: Default::default(),
            id_to_string,
            phantom_data: Default::default(),
        }
    }
    */
    pub (crate) fn get_shape_poses_internal<V: OVec<T>>(&self, state: &V) -> Cow<Vec<C::P<T>>> {
        let fk_res = self.forward_kinematics(state, None);

        self.get_shape_poses_from_fk_res(&fk_res)
    }
    pub fn get_shape_poses_from_fk_res(&self, fk_res: &FKResult<T, C::P<T>>) -> Cow<Vec<C::P<T>>> {
        let mut out = vec![];

        fk_res.link_poses.iter().enumerate().for_each(|(i, x)| {
            if let Some(pose) = x {
                let link = &self.links()[i];
                if link.is_present_in_model && link.convex_hull_file_path.is_some() {
                    out.push(pose.clone());
                }
            }
        });

        Cow::Owned(out)
    }
    #[inline(always)]
    pub fn parry_shape_scene(&self) -> &ORobotParryShapeScene<T, C, L> {
        &self.parry_shape_scene
    }
    pub fn parry_shape_scene_self_query<Q, V: OVec<T>>(&self, state: &V, query: &OwnedPairGroupQry<T, Q>, pair_selector: &ParryPairSelector, freeze: bool) -> <Q::OutputCategory as PairGroupQryOutputCategory>::Output<T, C::P<T>>
        where Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector>,
    {
        let shapes = self.parry_shape_scene.get_shapes();
        let p = self.get_shape_poses_internal(state);
        let pair_skips = self.parry_shape_scene.get_pair_skips();
        let pair_average_distances = self.parry_shape_scene.get_pair_average_distances();

        query.query(shapes, shapes, p.as_ref(), p.as_ref(), pair_selector, pair_skips, pair_average_distances, freeze)
    }
    pub fn parry_shape_scene_self_query_from_fk_res<Q>(&self, fk_res: &FKResult<T, C::P<T>>, query: &OwnedPairGroupQry<T, Q>, pair_selector: &ParryPairSelector, freeze: bool) -> <Q::OutputCategory as PairGroupQryOutputCategory>::Output<T, C::P<T>>
        where Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector>
    {
        let shapes = self.parry_shape_scene.get_shapes();
        let p = self.get_shape_poses_from_fk_res(fk_res);
        let pair_skips = self.parry_shape_scene.get_pair_skips();
        let pair_average_distances = self.parry_shape_scene.get_pair_average_distances();

        query.query(shapes, shapes, p.as_ref(), p.as_ref(), pair_selector, pair_skips, pair_average_distances, freeze)
    }
    #[inline(always)]
    pub fn get_dof_bounds(&self) -> Vec<(T, T)> {
        let mut out = vec![];
        /*
        self.joints().iter().for_each(|joint| {
           joint.limit().lower().iter().zip(joint.limit().upper().iter()).for_each(|(l, u)| {
               out.push( (*l, *u) )
            });
        });
        */
        self.dof_to_joint_and_sub_dof_idxs().iter().for_each(|(joint_idx, sub_dof_idx)| {
            let joint = &self.joints[*joint_idx];
            if joint.is_present_in_model {
                if joint.fixed_values().is_none() {
                    let l = joint.limit.lower()[*sub_dof_idx];
                    let u = joint.limit.upper()[*sub_dof_idx];
                    out.push((l, u));
                }
            }
        });

        out
    }
    #[inline(always)]
    pub fn get_dof_lower_bounds(&self) -> Vec<T> {
        let mut out = vec![];
        let dof_bounds = self.get_dof_bounds();
        dof_bounds.iter().for_each(|x| out.push(x.0));
        out
    }
    #[inline(always)]
    pub fn get_dof_upper_bounds(&self) -> Vec<T> {
        let mut out = vec![];
        let dof_bounds = self.get_dof_bounds();
        dof_bounds.iter().for_each(|x| out.push(x.1));
        out
    }
    #[inline(always)]
    pub fn sample_pseudorandom_state(&self) -> Vec<T> {
        let bounds = self.get_dof_bounds();
        SimpleSampler::uniform_samples(&bounds, None)
    }
    pub fn preprocess(&mut self, save: SaveRobot) {
        self.preprocess_robot_parry_shape_scene();
        self.has_been_preprocessed = true;

        match save {
            SaveRobot::Save(name) => {
                self.save_robot(name);
            }
            SaveRobot::DoNotSave => {  }
        }
    }
    pub fn parry_shape_scene_compute_average_distances(&mut self, save: SaveRobot, shape_average_dis_num_samples: Option<usize>) {
        let num_samples = match shape_average_dis_num_samples {
            None => { 1000 }
            Some(s) => { s }
        };
        let mut parry_shape_scene = self.parry_shape_scene.clone();
        parry_shape_scene.preprocess_shape_average_distances(self, num_samples);

        self.parry_shape_scene = parry_shape_scene;

        match save {
            SaveRobot::Save(name) => {
                self.save_robot(name);
            }
            SaveRobot::DoNotSave => {  }
        }
    }
    pub fn parry_shape_scene_compute_always_collision_pairs(&mut self, save: SaveRobot) {
        let mut parry_shape_scene = self.parry_shape_scene.clone();
        parry_shape_scene.preprocess_always_in_collision_states_pair_skips(self, 5000);

        self.parry_shape_scene = parry_shape_scene;

        match save {
            SaveRobot::Save(name) => {
                self.save_robot(name);
            }
            SaveRobot::DoNotSave => {  }
        }
    }
    pub fn parry_shape_scene_compute_never_collision_pairs(&mut self, save: SaveRobot) {
        let mut parry_shape_scene = self.parry_shape_scene.clone();
        parry_shape_scene.preprocess_never_in_collision_states_pair_skips(self, 5000);

        self.parry_shape_scene = parry_shape_scene;

        match save {
            SaveRobot::Save(name) => {
                self.save_robot(name);
            }
            SaveRobot::DoNotSave => {  }
        }
    }
    #[inline(always)]
    pub fn has_been_preprocessed(&self) -> bool {
        self.has_been_preprocessed
    }
    pub fn add_non_collision_state<V: OVec<T>>(&mut self, state: V, save_robot: SaveRobot) {
        if !self.non_collision_states.contains(&state.ovec_to_other_generic_category::<T, OVecCategoryVec>()) {
            self.non_collision_states.push(state.ovec_to_other_generic_category::<T, OVecCategoryVec>());
            self.set_non_collision_states_internal(save_robot);
        }
    }
    pub fn add_close_proximity_state<V: OVec<T>>(&mut self, state: V, threshold: T, save_robot: SaveRobot) {
        let r = self.clone();
        self.parry_shape_scene.add_close_proximity_states_pair_skips(&r, state.clone(), threshold );
        match save_robot {
            SaveRobot::Save(s) => { self.save_robot(s) }
            SaveRobot::DoNotSave => {}
        }
    }
    pub fn reset_non_collision_states(&mut self, save_robot: SaveRobot) {
        self.non_collision_states = vec![];
        // self.parry_shape_scene.add_non_collision_states_pair_skips(&self, &vec![]);

        self.set_non_collision_states_internal(save_robot);
    }
    pub fn reset_close_proximity_states(&mut self, save_robot: SaveRobot) {
        self.parry_shape_scene.clear_close_proximity_states_pair_skips();
        match save_robot {
            SaveRobot::Save(s) => { self.save_robot(s) }
            SaveRobot::DoNotSave => {}
        }
    }
    #[inline]
    pub fn get_ik_goal<V: OVec<T>>(&self, state: &V, link_idx: usize, ik_goal_mode: IKGoalMode<T, C>) -> C::P<T> {
        let fk_res = self.forward_kinematics(state, None);
        let pose = fk_res.get_link_pose(link_idx).as_ref().expect("error");
        return match ik_goal_mode {
            IKGoalMode::Absolute => { pose.clone() }
            IKGoalMode::LocalRelativeCombined { offset } => { pose.mul(&offset) }
            IKGoalMode::GlobalRelativeCombined { offset } => { offset.mul(pose) }
            IKGoalMode::LocalRelativeSeparate { offset } => {
                let new_t = pose.translation().o3dvec_add(offset.translation());
                let new_r = pose.rotation().mul(offset.rotation());
                C::P::from_translation_and_rotation(&new_t, &new_r)
            }
            IKGoalMode::GlobalRelativeSeparate { offset } => {
                let new_t = pose.translation().o3dvec_add(offset.translation());
                let new_r = offset.rotation().mul(pose.rotation());
                C::P::from_translation_and_rotation(&new_t, &new_r)
            }
        }
    }
    /*
    pub fn spawn_ik_differentiable_block<E: DerivativeMethodTrait>(&self, derivative_method_data: E::DerivativeMethodData) -> DifferentiableBlock<IKObjective<C, L>, E> {
        let robot1 = self.to_new_ad_type::<f64>();
        let robot2 = self.to_new_ad_type::<E::T>();

        let args1 = IKArgs {
            robot: Cow::Owned(robot1),
            goals: vec![],
        };

        let args2 = IKArgs {
            robot: Cow::Owned(robot2),
            goals: vec![],
        };

        DifferentiableBlock::new(args1, args2, derivative_method_data)
    }
    */
    /*
    pub fn get_ik_solver<E: DerivativeMethodTrait, OC: DiffBlockObjectiveOptimizerConstructorTrait>(&self, _derivative_method_data: E::DerivativeMethodData) -> Box<dyn IKOptimizer<D=IKObjective<C, L>, E=E, DataType=f64, OutputType = f64>> {
        /*
        let c = OC::optimizer_category();

        return match c {
            OptimizerCategory::SimpleOpEnEngineOptimizer => {
                let r1 = self.to_new_ad_type::<f64>();
                let r2 = r1.to_new_ad_type::<E::T>();

                let lower_bounds = r1.get_dof_lower_bounds();
                let upper_bounds = r1.get_dof_upper_bounds();

                let initial_condition = r1.sample_pseudorandom_state();

                let args1 = IKArgs {
                    robot: r1,
                    goals: vec![],
                };
                let args2 = IKArgs {
                    robot: r2,
                    goals: vec![],
                };

                let differentiable_block = DifferentiableBlock::<IKObjective<C, L>, E>::new(args1, args2, derivative_method_data);

                Box::new(SimpleOpEnEngineOptimizer {
                    differentiable_block,
                    initial_condition,
                    lower_bounds,
                    upper_bounds,
                    cache: Mutex::new(PANOCCache::new(self.num_dofs, 0.001, 3)),
                })
            }
        }
        */
        todo!()
    }
    */
    /*
    pub (crate) fn get_parry_pair_skips(&self) -> AHashMap<(u64, u64), ()> {
        let mut out = AHashMap::new();
        return match &self.robot_type {
            RobotType::Robot => {


                out
            }
            RobotType::RobotSet => {

                out
            }
        }
    }
    */
    fn set_non_collision_states_internal(&mut self, save_robot: SaveRobot) {
        let mut parry_shape_scene = self.parry_shape_scene.clone();
        parry_shape_scene.preprocess_non_collision_states_pair_skips(self, &self.non_collision_states);

        self.parry_shape_scene = parry_shape_scene;

        match save_robot {
            SaveRobot::Save(s) => { self.save_robot(s) }
            SaveRobot::DoNotSave => {}
        }
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
        self.set_link_convex_hull_mesh_file_paths();
        self.set_link_convex_decomposition_mesh_file_paths();
        // self.set_link_convex_decomposition_levels_mesh_file_paths();
        self.set_robot_parry_shape_scene();
    }
}
impl<T: AD, C: O3DPoseCategory, L: OLinalgCategory + 'static> ORobot<T, C, L> {
    fn set_link_and_joint_idxs(&mut self) {
        self.links.iter_mut().enumerate().for_each(|(i, x)| {
            x.link_idx = i;
            x.link_idx_in_sub_robot = i;
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
        let chain_info = compute_chain_info(&self.links, &self.joints);

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
                        target_path.append_file_location(&OAssetLocation::ChainOriginalMeshes { robot_name: &self.robot_name });
                        target_path.append(&file_check);
                        let exists = target_path.exists();

                        if !exists {
                            let asset_path = OPath::new_home_path();
                            oprint(&format!("searching for mesh {:?}", filepath), PrintMode::Println, PrintColor::Green);
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
                target_path.append_file_location(&OAssetLocation::ChainSTLMeshes { robot_name: &self.robot_name });
                target_path.append(&(filename.clone() + ".stl"));
                let exists = target_path.exists();

                if !exists {
                    oprint(&format!("saving stl version of {:?}", original_mesh_file_path.filename().unwrap()), PrintMode::Println, PrintColor::Green);
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
    fn set_link_convex_hull_mesh_file_paths(&mut self) {
        self.links.iter_mut().for_each(|link| {
            let stl_mesh_file = &link.stl_mesh_file_path;
            if let Some(stl_mesh_file) = stl_mesh_file {
                let filename = stl_mesh_file.filename().unwrap();

                let mut target_path = OStemCellPath::new_asset_path();
                target_path.append_file_location(&OAssetLocation::ChainConvexHulls { robot_name: &self.robot_name });
                target_path.append(&filename);

                let exists = target_path.exists();

                if !exists {
                    oprint(&format!("computing convex hull of {:?}", filename), PrintMode::Println, PrintColor::Green);
                    let convex_hull = stl_mesh_file.load_stl().to_trimesh().to_convex_hull();
                    convex_hull.save_to_stl(&target_path);
                }

                link.convex_hull_file_path = Some(target_path.clone());
            }
        });
    }
    fn set_link_convex_decomposition_mesh_file_paths(&mut self) {
        self.links.iter_mut().for_each(|link| {
            let stl_mesh_file = &link.stl_mesh_file_path;
            if let Some(stl_mesh_file) = stl_mesh_file {
                let filename = stl_mesh_file.filename_without_extension().unwrap();

                let mut target_path_stub = OStemCellPath::new_asset_path();
                target_path_stub.append_file_location(&OAssetLocation::LinkConvexDecomposition { robot_name: &self.robot_name, link_mesh_name: &filename });

                let exists = target_path_stub.exists();

                if !exists {
                    let convex_decomposition = stl_mesh_file.load_stl().to_trimesh().to_convex_decomposition(1);
                    oprint(&format!("computing convex decomposition of {:?}.  {:?} convex subcomponents found.", filename, convex_decomposition.len()), PrintMode::Println, PrintColor::Green);

                    convex_decomposition.iter().enumerate().for_each(|(i, trimesh)| {
                        let mut target_path = target_path_stub.clone();
                        target_path.append(&(filename.clone() + "_" + i.to_string().as_str() + ".stl"));
                        trimesh.save_to_stl(&target_path);
                    });
                }

                let files = target_path_stub.get_all_items_in_directory_as_paths(false, false);
                link.convex_decomposition_file_paths = files.clone();
            }
        });
    }
    #[allow(dead_code)]
    fn set_link_convex_decomposition_levels_mesh_file_paths(&mut self) {
        let max_num_convex_hulls = vec![1, 2, 5, 10, 20, 10000];
        for (level, max_num) in max_num_convex_hulls.iter().enumerate() {
            self.links.iter_mut().for_each(|link| {
                let stl_mesh_file = &link.stl_mesh_file_path;
                if let Some(stl_mesh_file) = stl_mesh_file {
                    let filename = stl_mesh_file.filename_without_extension().unwrap();

                    let mut target_path_stub = OStemCellPath::new_asset_path();
                    target_path_stub.append_file_location(&OAssetLocation::LinkConvexDecompositionLevel { robot_name: &self.robot_name, level, link_mesh_name: &filename });

                    let exists = target_path_stub.exists();

                    if !exists {
                        let convex_decomposition = stl_mesh_file.load_stl().to_trimesh().to_convex_decomposition(*max_num);
                        oprint(&format!("computing convex decomposition of {:?} at level {:?}.  {:?} convex subcomponents found.", filename, level, convex_decomposition.len()), PrintMode::Println, PrintColor::Green);

                        convex_decomposition.iter().enumerate().for_each(|(i, trimesh)| {
                            let mut target_path = target_path_stub.clone();
                            target_path.append(&(filename.clone() + "_" + i.to_string().as_str() + ".stl"));
                            trimesh.save_to_stl(&target_path);
                        });
                    }

                    let files = target_path_stub.get_all_items_in_directory_as_paths(false, false);
                    // link.convex_decomposition_file_paths = files.clone();
                    link.convex_decomposition_levels_file_paths.push(files.clone());
                }
            });
        }
    }
    fn set_robot_parry_shape_scene(&mut self) {
        self.parry_shape_scene = ORobotParryShapeScene::new(self);
    }
    fn preprocess_robot_parry_shape_scene(&mut self) {
        let mut parry_shape_scene = ORobotParryShapeScene::new(self);

        // parry_shape_scene.add_non_collision_states_pair_skips::<Vec<T>>(self, &self.non_collision_states);
        parry_shape_scene.preprocess_non_collision_states_pair_skips(self, &self.non_collision_states);
        parry_shape_scene.preprocess_always_in_collision_states_pair_skips(self, 5000);
        parry_shape_scene.preprocess_never_in_collision_states_pair_skips(self, 5000);
        parry_shape_scene.preprocess_shape_average_distances(self, 1000);

        self.parry_shape_scene = parry_shape_scene;
    }
}
impl<C: O3DPoseCategory, L: OLinalgCategory> ORobot<f64, C, L> {
    pub fn get_ik_differentiable_block<'a, E, FQ, Q>(&'a self, derivative_method: E, filter_query: OwnedPairGroupQry<'a, f64, FQ>, distance_query: OwnedPairGroupQry<'a, f64, Q>, constant_selector: Option<ParryPairSelector>, init_state: &[f64], ik_goal_link_idxs: Vec<usize>, linf_dis_cutoff: f64, dis_filter_cutoff: f64, ee_matching_weight: f64, self_collision_avoidance_weight: f64, min_vel_weight: f64, min_acc_weight: f64, min_jerk_weight: f64) -> DifferentiableBlock<'a, DifferentiableFunctionClassIKObjective<C, L, FQ, Q>, E>
        where E: DerivativeMethodTrait,
              FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
              Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=ToParryProximityOutputCategory> {

        let last_proximity_filter_state: Rc<RefCell<Option<Vec<f64>>>> = Rc::new(RefCell::new(None));
        let filter_output: Rc<RefCell<Option<ParryFilterOutput>>> = Rc::new(RefCell::new(None));

        let f2 = self.get_ik_objective_function(Cow::Owned(self.to_new_ad_type::<E::T>()), filter_query.clone(), distance_query.clone(), constant_selector.clone(), init_state, ik_goal_link_idxs.clone(), linf_dis_cutoff, dis_filter_cutoff, ee_matching_weight, self_collision_avoidance_weight, min_vel_weight, min_acc_weight, min_jerk_weight, last_proximity_filter_state.clone(), filter_output.clone());
        let f1= self.get_ik_objective_function(Cow::Borrowed(self), filter_query, distance_query, constant_selector, init_state, ik_goal_link_idxs, linf_dis_cutoff, dis_filter_cutoff, ee_matching_weight, self_collision_avoidance_weight, min_vel_weight, min_acc_weight, min_jerk_weight, last_proximity_filter_state.clone(), filter_output.clone());

        DifferentiableBlockIKObjective::new(derivative_method, f1, f2)
    }
    pub fn get_look_at_differentiable_block<'a, E, FQ, Q>(&'a self, derivative_method: E, filter_query: OwnedPairGroupQry<'a, f64, FQ>, distance_query: OwnedPairGroupQry<'a, f64, Q>, constant_selector: Option<ParryPairSelector>, init_state: &[f64], ik_goal_link_idxs: Vec<usize>, looker_link: usize, looker_forward_axis: AxisDirection, looker_side_axis: AxisDirection, look_at_target: LookAtTarget<f64, O3DVecCategoryArr>, linf_dis_cutoff: f64, dis_filter_cutoff: f64, ee_matching_weight: f64, self_collision_avoidance_weight: f64, min_vel_weight: f64, min_acc_weight: f64, min_jerk_weight: f64, look_at_weight: f64, roll_prevention_weight: f64) -> DifferentiableBlock<'a, DifferentiableFunctionClassLookAt<C, L, FQ, Q>, E>
        where E: DerivativeMethodTrait,
              FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
              Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=ToParryProximityOutputCategory> {
        let last_proximity_filter_state: Rc<RefCell<Option<Vec<f64>>>> = Rc::new(RefCell::new(None));
        let filter_output: Rc<RefCell<Option<ParryFilterOutput>>> = Rc::new(RefCell::new(None));

        let f2 = self.get_look_at_objective_function(Cow::Owned(self.to_new_ad_type::<E::T>()), filter_query.clone(), distance_query.clone(), constant_selector.clone(), init_state, ik_goal_link_idxs.clone(), looker_link, looker_forward_axis.clone(), looker_side_axis.clone(), look_at_target.clone(), linf_dis_cutoff, dis_filter_cutoff, ee_matching_weight, self_collision_avoidance_weight, min_vel_weight, min_acc_weight, min_jerk_weight, look_at_weight, roll_prevention_weight, last_proximity_filter_state.clone(), filter_output.clone());
        let f1 = self.get_look_at_objective_function(Cow::Borrowed(self), filter_query, distance_query, constant_selector, init_state, ik_goal_link_idxs, looker_link, looker_forward_axis.clone(), looker_side_axis.clone(), look_at_target.clone(), linf_dis_cutoff, dis_filter_cutoff, ee_matching_weight, self_collision_avoidance_weight, min_vel_weight, min_acc_weight, min_jerk_weight, look_at_weight, roll_prevention_weight, last_proximity_filter_state.clone(), filter_output.clone());

        DifferentiableBlock::new(derivative_method, f1, f2)
    }
}
/// Objective Functions
impl<T: AD, C: O3DPoseCategory, L: OLinalgCategory> ORobot<T, C, L > {
    pub fn get_ik_objective_function<'a, T1, FQ, Q>(&'a self, robot: Cow<'a, ORobot<T1, C, L>>, filter_query: OwnedPairGroupQry<'a, f64, FQ>, distance_query: OwnedPairGroupQry<'a, f64, Q>, constant_selector: Option<ParryPairSelector>, init_state: &[f64], ik_goal_link_idxs: Vec<usize>, linf_dis_cutoff: f64, dis_filter_cutoff: f64, ee_matching_weight: f64, self_collision_avoidance_weight: f64, min_vel_weight: f64, min_acc_weight: f64, min_jerk_weight: f64, last_proximity_filter_state: Rc<RefCell<Option<Vec<f64>>>>, filter_output: Rc<RefCell<Option<ParryFilterOutput>>>) -> DifferentiableFunctionIKObjective<T1, C, L, FQ, Q>
        where T1: AD,
              FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
              Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=ToParryProximityOutputCategory>
    {
        let fk_res = self.forward_kinematics(&init_state.to_vec().ovec_to_other_ad_type::<T>(), None);

        let mut ik_goals: Vec<IKGoal<T, C::P<T>>> = vec![];
        ik_goal_link_idxs.iter().for_each(|x| { ik_goals.push(IKGoal::new(*x, fk_res.get_link_pose(*x).as_ref().expect("error").clone(), T::constant(1.0))); });

        let f = DifferentiableFunctionIKObjective::new(robot, ik_goals.to_new_generic_types::<T1, C>(), init_state.to_vec().ovec_to_other_ad_type::<T1>(), filter_query.to_new_ad_type::<T1>(), distance_query.to_new_ad_type::<T1>(), constant_selector, T1::constant(dis_filter_cutoff), linf_dis_cutoff, last_proximity_filter_state.clone(), filter_output.clone(), T1::constant(ee_matching_weight), T1::constant(self_collision_avoidance_weight), T1::constant(min_vel_weight), T1::constant(min_acc_weight), T1::constant(min_jerk_weight));

        f
    }
    pub fn get_look_at_objective_function<'a, T1, FQ, Q>(&'a self, robot: Cow<'a, ORobot<T1, C, L>>, filter_query: OwnedPairGroupQry<'a, f64, FQ>, distance_query: OwnedPairGroupQry<'a, f64, Q>, constant_selector: Option<ParryPairSelector>, init_state: &[f64], ik_goal_link_idxs: Vec<usize>, looker_link: usize, looker_forward_axis: AxisDirection, looker_side_axis: AxisDirection, look_at_target: LookAtTarget<f64, O3DVecCategoryArr>, linf_dis_cutoff: f64, dis_filter_cutoff: f64, ee_matching_weight: f64, self_collision_avoidance_weight: f64, min_vel_weight: f64, min_acc_weight: f64, min_jerk_weight: f64, look_at_weight: f64, roll_prevention_weight: f64, last_proximity_filter_state: Rc<RefCell<Option<Vec<f64>>>>, filter_output: Rc<RefCell<Option<ParryFilterOutput>>>) -> DifferentiableFunctionLookAt<T1, C, L, FQ, Q>
        where T1: AD,
              FQ: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=PairGroupQryOutputCategoryParryFilter>,
              Q: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=ParryPairSelector, OutputCategory=ToParryProximityOutputCategory> {
        let ik_objective = self.get_ik_objective_function(robot, filter_query, distance_query, constant_selector, init_state, ik_goal_link_idxs, linf_dis_cutoff, dis_filter_cutoff, ee_matching_weight, self_collision_avoidance_weight, min_vel_weight, min_acc_weight, min_jerk_weight, last_proximity_filter_state, filter_output);

        DifferentiableFunctionLookAt::new(ik_objective, looker_link, looker_forward_axis, looker_side_axis, look_at_target.to_new_ad_type::<T1>(), T1::constant(look_at_weight), T1::constant(roll_prevention_weight))
    }
}
impl<T: AD, C: O3DPoseCategory + 'static, L: OLinalgCategory + 'static> AsRobotTrait<T, C, L> for ORobot<T, C, L> {
    #[inline(always)]
    fn as_robot(&self) -> &ORobot<T, C, L> {
        self
    }
}
impl<T: AD, C: O3DPoseCategory, L: OLinalgCategory + 'static> Debug for ORobot<T, C, L> {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let mut s = "".to_string();

        s += "{{\n";
        s += &format!("  ORobot\n");
        s += &format!("  Robot name: {}\n", self.robot_name);
        s += &format!("  Num DOFS: {}\n", self.num_dofs);
        s += "//////////////////////////////////////////////////////////////////////////////////////";
        s += "\n";
        s += &format!("  Links:\n");
        let binding = self.get_links_string();
        s += &tab!(binding);
        s += "\n";
        s += "//////////////////////////////////////////////////////////////////////////////////////";
        s += "\n";
        s += &format!("  Joints:\n");
        let binding = self.get_joints_string();
        s += &tab!(binding);
        s += "\n";
        s += "//////////////////////////////////////////////////////////////////////////////////////";
        s += "\n}}";

        f.write_str(&s)?;
        Ok(())
    }
}
impl<T: AD, C: O3DPoseCategory, L: OLinalgCategory + 'static> ShapeSceneTrait<T, C::P<T>> for ORobot<T, C, L> {
    type ShapeType = OParryShape<T, C::P<T>>;
    type GetPosesInput<'a> = &'a [T] where Self: 'a;
    type PairSkipsType = AHashMapWrapper<(u64, u64), Vec<SkipReason>>;

    #[inline(always)]
    fn get_shapes(&self) -> &Vec<Self::ShapeType> {
        self.parry_shape_scene.get_shapes()
    }

    #[inline(always)]
    fn get_shape_poses<'a>(&'a self, input: &Self::GetPosesInput<'a>) -> Cow<Vec<C::P<T>>> {
        self.get_shape_poses_internal(&input.to_vec())
    }

    #[inline(always)]
    fn get_pair_skips(&self) -> &Self::PairSkipsType {
        self.parry_shape_scene.get_pair_skips()
    }

    #[inline(always)]
    fn shape_id_to_shape_str(&self, id: u64) -> String {
        self.parry_shape_scene.shape_id_to_shape_str(id)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum RobotType {
    Robot,
    RobotSet
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FKResult<T: AD, P: O3DPose<T>> {
    #[serde(deserialize_with = "Vec::<Option::<P>>::deserialize")]
    pub (crate) link_poses: Vec<Option<P>>,
    _phantom_data: PhantomData<T>
}
impl<T: AD, P: O3DPose<T>> FKResult<T, P> {
    #[inline]
    pub fn link_poses(&self) -> &Vec<Option<P>> {
        &self.link_poses
    }
    #[inline]
    pub fn get_link_pose(&self, link_idx: usize) -> &Option<P> {
        &self.link_poses[link_idx]
    }
}

#[derive(Clone, Debug)]
pub enum SaveRobot<'a> {
    Save(Option<&'a str>),
    DoNotSave
}

pub enum IKGoalMode<T: AD, C: O3DPoseCategory> {
    Absolute,
    LocalRelativeCombined { offset: C::P<T> },
    GlobalRelativeCombined { offset: C::P<T> },
    LocalRelativeSeparate { offset: C::P<T> },
    GlobalRelativeSeparate { offset: C::P<T> }
}

