use ad_trait::AD;
use bevy::pbr::StandardMaterial;
use bevy::prelude::*;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_linalg::OLinalgTrait;
use optima_robotics::robot::ORobot;
use optima_robotics::robotics_traits::ChainableTrait;
use crate::optima_bevy_utils::file::get_asset_path_str_from_ostemcellpath;

pub struct RobotActions;
impl RobotActions {
    pub fn action_spawn_robot_links<T: AD, P: O3DPose<T>, L: OLinalgTrait>(robot: &BevyORobot<T, P, L>,
                                                                           commands: &mut Commands,
                                                                           asset_server: &AssetServer,
                                                                           materials: &mut Assets<StandardMaterial>) {
        let robot = &robot.0;

        robot.chain_wrappers().iter().enumerate().for_each(|(chain_idx, chain_wrapper)| {
            let chain = chain_wrapper.chain();
            chain.links().iter().enumerate().for_each(|(link_idx, link)| {
                if let Some(stl_mesh_file_path) = link.stl_mesh_file_path() {
                    let asset_file_path = get_asset_path_str_from_ostemcellpath(stl_mesh_file_path);

                    commands.spawn(PbrBundle {
                        mesh: asset_server.load(&asset_file_path),
                        material: materials.add(StandardMaterial::default()),
                        transform: Default::default(),
                        global_transform: Default::default(),
                        visibility: Visibility::Visible,
                        ..Default::default()
                    });
                }
            });
        });
    }
}

pub struct RobotSystems;
impl RobotSystems {
    pub fn system_spawn_robot_links<T: AD, P: O3DPose<T> + 'static, L: OLinalgTrait + 'static>(robot: Res<BevyORobot<T, P, L>>,
                                                                                               mut commands: Commands,
                                                                                               asset_server: Res<AssetServer>,
                                                                                               mut materials: ResMut<Assets<StandardMaterial>>) {
        RobotActions::action_spawn_robot_links(& *robot, &mut commands, & *asset_server, &mut *materials);
    }
}


#[derive(Resource)]
pub struct BevyORobot<T: AD, P: O3DPose<T> + Send, L: OLinalgTrait>(pub ORobot<T, P, L>);
