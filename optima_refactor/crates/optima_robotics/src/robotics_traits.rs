use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_3d_spatial::optima_3d_rotation::ScaledAxis;
use optima_linalg::vecs_and_mats::{OLinalgTrait, OVec};
use optima_utils::arr_storage::ImmutArrTraitRaw;
use crate::robotics_components::{ChainInfo, OJointType, OPose};

pub trait JointTrait<T: AD, P: O3DPose<T>> {
    fn joint_idx(&self) -> usize;
    fn parent_link_idx(&self) -> usize;
    fn child_link_idx(&self) -> usize;
    fn joint_type(&self) -> &OJointType;
    fn axis(&self) -> &[T; 3];
    fn origin(&self) -> &OPose<T, P>;
    fn get_variable_transform_from_joint_values_subslice(&self, joint_values_subslice: &[T]) -> P {
        return match &self.joint_type() {
            OJointType::Revolute => {
                assert_eq!(joint_values_subslice.len(), 1);
                let axis = self.axis();
                let axis = axis.scalar_mul(&joint_values_subslice[0]);
                P::from_translation_and_rotation_constructor(&[T::zero(), T::zero(), T::zero()], &ScaledAxis(axis))
            }
            OJointType::Continuous => {
                assert_eq!(joint_values_subslice.len(), 1);
                let axis = self.axis();
                let axis = axis.scalar_mul(&joint_values_subslice[0]);
                P::from_translation_and_rotation_constructor(&[T::zero(), T::zero(), T::zero()], &ScaledAxis(axis))
            }
            OJointType::Prismatic => {
                assert_eq!(joint_values_subslice.len(), 1);
                let axis = self.axis();
                let axis = axis.scalar_mul(&joint_values_subslice[0]);
                P::from_translation_and_rotation_constructor(&axis, &[T::zero(), T::zero(), T::zero()])
            }
            OJointType::Fixed => {
                P::identity()
            }
            OJointType::Floating => {
                assert_eq!(joint_values_subslice.len(), 6);
                P::from_translation_and_rotation_constructor(&[joint_values_subslice[0], joint_values_subslice[1], joint_values_subslice[2]], &ScaledAxis([joint_values_subslice[3], joint_values_subslice[4], joint_values_subslice[5]]))
            }
            OJointType::Planar => {
                assert_eq!(joint_values_subslice.len(), 2);
                P::from_translation_and_rotation_constructor(&[joint_values_subslice[0], joint_values_subslice[1], T::zero()], &[T::zero(), T::zero(), T::zero()])
            }
            OJointType::Spherical => {
                assert_eq!(joint_values_subslice.len(), 3);
                P::from_translation_and_rotation_constructor(&[T::zero(), T::zero(), T::zero()], &ScaledAxis([joint_values_subslice[0], joint_values_subslice[1], joint_values_subslice[2]]))
            }
        }
    }
    fn get_joint_fixed_offset_transform(&self) -> &P {
        self.origin().pose()
    }
    fn get_num_dofs(&self) -> usize;
}

pub trait ChainableTrait<T: AD, P: O3DPose<T>> {
    type LinkType;
    type JointType : JointTrait<T, P>;

    fn links(&self) -> &Vec<Self::LinkType>;
    fn joints(&self) -> &Vec<Self::JointType>;
    fn compute_chain_info(&self) -> ChainInfo {
        let num_links = self.links().len();
        let num_joints = self.joints().len();

        let mut joint_parent_link = vec![ usize::default(); num_joints ];
        let mut joint_child_link = vec![ usize::default(); num_joints ];
        for joint in self.joints().iter() {
            let joint_idx = joint.joint_idx();
            joint_parent_link[joint_idx] = joint.parent_link_idx();
            joint_child_link[joint_idx] = joint.child_link_idx();
        }

        let mut link_parent_joint = vec![ None; num_links ];
        let mut link_children_joints = vec![vec![]; num_links ];
        for joint in self.joints().iter() {
            let joint_idx = joint.joint_idx();
            let parent_link_idx = joint.parent_link_idx();
            let child_link_idx = joint.child_link_idx();

            link_parent_joint[child_link_idx] = Some(joint_idx);
            link_children_joints[parent_link_idx].push(joint_idx);
        }

        let mut link_parent_link = vec![ None; num_links ];
        for (link_idx, joint_idx) in link_parent_joint.iter().enumerate() {
            match joint_idx {
                None => { }
                Some(joint_idx) => {
                    let parent_link_idx = joint_parent_link[*joint_idx];
                    link_parent_link[link_idx] = Some(parent_link_idx);
                }
            }
        }

        let mut link_children_links = vec![ vec![]; num_links ];
        for (link_idx, joint_idxs) in link_children_joints.iter().enumerate() {
            for joint_idx in joint_idxs {
                link_children_links[link_idx].push(joint_child_link[*joint_idx]);
            }
        }

        let mut base_link_idx = None;
        for (link_idx, joint_idx) in link_parent_joint.iter().enumerate() {
            match joint_idx {
                None => {
                    match base_link_idx {
                        None => { base_link_idx = Some(link_idx) }
                        Some(_) => { panic!("multiple base links found.  {} and {}.", base_link_idx.unwrap(), link_idx); }
                    }
                }
                Some(_) => {}
            }
        }
        let base_link_idx = base_link_idx.expect("no base link found.");

        let mut kinematic_hierarchy = vec![ vec![ base_link_idx ] ];
        let mut curr_layer_idx = 0;
        'l: loop {
            let mut layer_to_add = vec![];
            let curr_layer = kinematic_hierarchy.get(curr_layer_idx).unwrap();
            for link_idx in curr_layer.iter() {
                let children_link_idxs = &link_children_links[*link_idx];
                for child_link_idx in children_link_idxs {
                    layer_to_add.push(*child_link_idx);
                }
            }
            if layer_to_add.is_empty() { break 'l; }
            else { kinematic_hierarchy.push(layer_to_add); curr_layer_idx += 1; }
        }

        let mut link_connection_paths = vec![ vec![ None; num_links  ] ; num_links];
        let mut path_stack = vec![ vec![ base_link_idx ] ];
        'l: loop {
            if path_stack.is_empty() { break 'l; }
            let curr_path = path_stack.pop().unwrap();
            let curr_path_len = curr_path.len();
            if curr_path_len > 1 {
                for i in 0..curr_path_len-1 {
                    let subslice = curr_path.subslice(i, curr_path_len).to_vec();
                    assert!(subslice.len() > 1);
                    let start_idx = subslice[0];
                    let end_idx = subslice[subslice.len() - 1];
                    link_connection_paths[start_idx][end_idx] = Some(subslice);
                }
            }

            let last_entry = curr_path[curr_path.len()-1];
            let children_links = &link_children_links[last_entry];
            for child_link in children_links {
                let mut curr_path_clone = curr_path.clone();
                curr_path_clone.push(*child_link);
                path_stack.push(curr_path_clone);
            }
        }

        ChainInfo {
            joint_parent_link,
            joint_child_link,
            link_parent_joint,
            link_children_joints,
            link_parent_link,
            link_children_links,
            base_link_idx,
            kinematic_hierarchy,
            link_connection_paths,
        }
    }
}

pub trait OForwardKinematicsTrait<T: AD, P: O3DPose<T>> {
    fn forward_kinematics<V: OVec<T>>(&self, state: &V, base_offset: Option<&P>) -> Vec<Vec<Option<P>>>;
    fn forward_kinematics_floating_chain<V: OVec<T>>(&self, state: &V, start_link_idx: usize, end_link_idx: usize, base_offset: Option<&P>) -> Vec<Vec<Option<P>>>;
}

pub trait OJacobianTrait<T: AD, P: O3DPose<T>, L: OLinalgTrait> : OForwardKinematicsTrait<T, P> {
    fn jacobian<V: OVec<T>>(&self, state: &V, start_link_idx: Option<usize>, end_link_idx: usize) -> L::MatType<T>;
}

/*
#[derive(Clone, Debug)]
pub enum JacobianMode {
    Full, Translational, Rotational
}

#[derive(Clone, Debug)]
pub enum JacobianEndPoint<T: AD, H: O3DVec<T>> {
    Link,
    LocalOffset(H),
    GlobalOffset(H),
    InertialOrigin,
    _PhantomData(PhantomData<T>)
}
*/