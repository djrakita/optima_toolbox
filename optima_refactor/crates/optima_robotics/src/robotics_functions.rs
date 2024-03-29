use ad_trait::AD;
use optima_3d_spatial::optima_3d_pose::O3DPoseCategory;
use optima_misc::arr_storage::ImmutArrTraitRaw;
use crate::robotics_components::ChainInfo;
use crate::robotics_traits::JointTrait;

pub fn compute_chain_info<T: AD, C: O3DPoseCategory + 'static, L, J: JointTrait<T, C>>(chainable_link_objects: &Vec<L>, chainable_joint_objects: &Vec<J>) -> ChainInfo {
    let num_links = chainable_link_objects.len();
    let num_joints = chainable_joint_objects.len();

    let mut joint_parent_link = vec![usize::default(); num_joints];
    let mut joint_child_link = vec![usize::default(); num_joints];
    for joint in chainable_joint_objects.iter() {
        let joint_idx = joint.joint_idx();
        joint_parent_link[joint_idx] = joint.parent_link_idx();
        joint_child_link[joint_idx] = joint.child_link_idx();
    }

    let mut link_parent_joint = vec![None; num_links];
    let mut link_children_joints = vec![vec![]; num_links];
    for joint in chainable_joint_objects.iter() {
        let joint_idx = joint.joint_idx();
        let parent_link_idx = joint.parent_link_idx();
        let child_link_idx = joint.child_link_idx();

        link_parent_joint[child_link_idx] = Some(joint_idx);
        link_children_joints[parent_link_idx].push(joint_idx);
    }

    let mut link_parent_link = vec![None; num_links];
    for (link_idx, joint_idx) in link_parent_joint.iter().enumerate() {
        match joint_idx {
            None => {}
            Some(joint_idx) => {
                let parent_link_idx = joint_parent_link[*joint_idx];
                link_parent_link[link_idx] = Some(parent_link_idx);
            }
        }
    }

    let mut link_children_links = vec![vec![]; num_links];
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

    let mut kinematic_hierarchy = vec![vec![base_link_idx]];
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
        if layer_to_add.is_empty() { break 'l; } else {
            kinematic_hierarchy.push(layer_to_add);
            curr_layer_idx += 1;
        }
    }

    let mut link_connection_paths = vec![vec![None; num_links]; num_links];
    let mut path_stack = vec![vec![base_link_idx]];
    'l: loop {
        if path_stack.is_empty() { break 'l; }
        let curr_path = path_stack.pop().unwrap();
        let curr_path_len = curr_path.len();
        if curr_path_len > 1 {
            for i in 0..curr_path_len - 1 {
                let subslice = curr_path.subslice(i, curr_path_len).to_vec();
                assert!(subslice.len() > 1);
                let start_idx = subslice[0];
                let end_idx = subslice[subslice.len() - 1];
                link_connection_paths[start_idx][end_idx] = Some(subslice);
            }
        }

        let last_entry = curr_path[curr_path.len() - 1];
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
