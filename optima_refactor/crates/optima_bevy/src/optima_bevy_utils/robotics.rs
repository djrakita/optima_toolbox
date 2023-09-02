use ad_trait::AD;
use bevy::pbr::StandardMaterial;
use bevy::prelude::*;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::OLinalgTrait;
use optima_robotics::chain::{ChainFKResult, OChain};
use optima_robotics::robot::ORobot;
use optima_robotics::robotics_traits::ChainableTrait;
use crate::optima_bevy_utils::file::get_asset_path_str_from_ostemcellpath;
use crate::optima_bevy_utils::transform::TransformUtils;
use crate::OptimaBevyTrait;

pub struct RoboticsActions;
impl RoboticsActions {
    pub fn action_spawn_chain_links_as_stl_meshes<T: AD, P: O3DPose<T>, L: OLinalgTrait>(chain: &OChain<T, P, L>,
                                                                                         fk_res: &ChainFKResult<T, P>,
                                                                                         commands: &mut Commands,
                                                                                         asset_server: &AssetServer,
                                                                                         materials: &mut Assets<StandardMaterial>,
                                                                                         robot_instance_idx: usize,
                                                                                         chain_idx: usize) {
        chain.links().iter().enumerate().for_each(|(link_idx, link)| {
            if link.is_present_in_model() {
                let stl_mesh_file_path = link.stl_mesh_file_path();
                if let Some(stl_mesh_file_path) = stl_mesh_file_path {
                    let asset_path_str = get_asset_path_str_from_ostemcellpath(&stl_mesh_file_path);
                    let link_pose = fk_res.get_link_pose(link_idx);
                    if let Some(link_pose) = link_pose {
                        let transform = TransformUtils::util_convert_3d_pose_to_y_up_bevy_transform(link_pose);

                        commands.spawn(PbrBundle {
                            mesh: asset_server.load(&asset_path_str),
                            material: materials.add(StandardMaterial::default()),
                            transform,
                            ..Default::default()
                        }).insert(LinkMeshID {
                            robot_instance_idx,
                            chain_idx,
                            link_idx,
                        });
                    }
                }
            }
        });
    }
}

pub struct RoboticsSystems;
impl RoboticsSystems {
    pub fn system_spawn_robot_links_as_stl_meshes<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static> (robot: Res<BevyORobot<T, P, L>>,
                                                                                                             mut commands: Commands,
                                                                                                             asset_server: Res<AssetServer>,
                                                                                                             mut materials: ResMut<Assets<StandardMaterial>>) {
        let robot = &robot.0;
        let num_dofs = robot.num_dofs();
        let robot_fk_res = robot.forward_kinematics(&vec![T::zero(); num_dofs], None);

        robot.chain_wrappers().iter().enumerate().for_each(|(chain_idx, chain_wrapper)| {
            let chain_fk_res = robot_fk_res.get_chain_fk_result(chain_idx).as_ref().unwrap();
            RoboticsActions::action_spawn_chain_links_as_stl_meshes(chain_wrapper.chain(), chain_fk_res, &mut commands, & *asset_server, &mut *materials, 0, chain_idx);
        });
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait BevyRoboticsTrait {
    fn bevy_display(&self);
}

impl<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static> BevyRoboticsTrait for ORobot<T, P, L> {
    fn bevy_display(&self) {
        App::new()
            .optima_bevy_base()
            .optima_bevy_pan_orbit_camera()
            .optima_bevy_starter_lights()
            .optima_bevy_spawn_robot(self.clone())
            .run();
    }
}

impl<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static> BevyRoboticsTrait for OChain<T, P, L> {
    fn bevy_display(&self) {
        let robot = ORobot::new_from_single_chain(self.clone());
        robot.bevy_display();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Component)]
pub struct LinkMeshID {
    pub robot_instance_idx: usize,
    pub chain_idx: usize,
    pub link_idx: usize
}

#[derive(Resource)]
pub struct BevyORobot<T: AD, P: O3DPose<T> + Send, L: OLinalgTrait>(pub ORobot<T, P, L>);
