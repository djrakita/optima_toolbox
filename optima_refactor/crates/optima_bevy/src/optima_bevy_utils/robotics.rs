use ad_trait::AD;
use bevy::pbr::StandardMaterial;
use bevy::prelude::*;
use bevy_egui::egui::Ui;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryTrait};
use optima_bevy_egui::OEguiEngineWrapper;
use optima_linalg::{OLinalgCategoryTrait, OVec};
use optima_robotics::chain::{ChainFKResult, OChain};
use optima_robotics::robot::ORobot;
use optima_robotics::robotics_traits::ChainableTrait;
use crate::optima_bevy_utils::file::get_asset_path_str_from_ostemcellpath;
use crate::optima_bevy_utils::transform::TransformUtils;
use crate::OptimaBevyTrait;

pub struct RoboticsActions;
impl RoboticsActions {
    pub fn action_spawn_robot_as_stl_meshes<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait>(robot: &ORobot<T, C, L>,
                                                                                                     commands: &mut Commands,
                                                                                                     asset_server: &AssetServer,
                                                                                                     materials: &mut Assets<StandardMaterial>,
                                                                                                     robot_instance_idx: usize) {
        let num_dofs = robot.num_dofs();
        let robot_fk_res = robot.forward_kinematics(&vec![T::zero(); num_dofs], None);

        robot.chain_wrappers().iter().enumerate().for_each(|(chain_idx, chain_wrapper)| {
            let chain_fk_res = robot_fk_res.get_chain_fk_result(chain_idx).as_ref().unwrap();
            Self::action_spawn_chain_links_as_stl_meshes(chain_wrapper.chain(), chain_fk_res, commands, asset_server, materials, robot_instance_idx, chain_idx);
        });
    }
    pub fn action_spawn_chain_links_as_stl_meshes<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait>(chain: &OChain<T, C, L>,
                                                                                                           fk_res: &ChainFKResult<T, C::P<T>>,
                                                                                                           commands: &mut Commands,
                                                                                                           asset_server: &AssetServer,
                                                                                                           materials: &mut Assets<StandardMaterial>,
                                                                                                           robot_instance_idx: usize,
                                                                                                           chain_idx: usize) {
        chain.chainable_link_objects().iter().enumerate().for_each(|(link_idx, link)| {
            if link.is_present_in_model() {
                let stl_mesh_file_path = link.stl_mesh_file_path();
                if let Some(stl_mesh_file_path) = stl_mesh_file_path {
                    let asset_path_str = get_asset_path_str_from_ostemcellpath(&stl_mesh_file_path);
                    let link_pose = fk_res.get_link_pose(link_idx);
                    if let Some(link_pose) = link_pose {
                        let visual_offset = link.visual()[0].origin().pose();
                        let link_pose = link_pose.mul(visual_offset);

                        let transform = TransformUtils::util_convert_3d_pose_to_y_up_bevy_transform(&link_pose);

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
    pub fn action_set_state_of_robot<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait, V: OVec<T>>(robot: &ORobot<T, C, L>,
                                                                                                          state: &V,
                                                                                                          robot_instance_idx: usize,
                                                                                                          query: &mut Query<(&LinkMeshID, &mut Transform)>) {
        let robot_fk_res = robot.forward_kinematics(state, None);

        for (link_mesh_id, mut transform) in query.iter_mut() {
            let link_mesh_id: &LinkMeshID = &link_mesh_id;
            let transform: &mut Transform = &mut transform;

            if link_mesh_id.robot_instance_idx == robot_instance_idx {
                let chain_idx = link_mesh_id.chain_idx;
                let link_idx = link_mesh_id.link_idx;
                let link = &robot.chain_wrappers()[chain_idx].chain().chainable_link_objects()[link_idx];
                let pose = robot_fk_res.get_chain_fk_result(chain_idx).as_ref().unwrap().get_link_pose(link_idx).as_ref().unwrap();
                let visual_offset = link.visual()[0].origin().pose();
                *transform = TransformUtils::util_convert_3d_pose_to_y_up_bevy_transform(&(pose.mul(visual_offset)));
            }
        }
    }
    pub fn action_robot_joint_sliders_egui<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait>(_robot: &ORobot<T, C, L>,
                                                                                                    _egui_engine: &Res<OEguiEngineWrapper>,
                                                                                                    _ui: &mut Ui) {
        todo!()
    }
}

pub struct RoboticsSystems;
impl RoboticsSystems {
    pub fn system_spawn_robot_links_as_stl_meshes<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static>(robot: Res<BevyORobot<T, C, L>>,
                                                                                                                               mut commands: Commands,
                                                                                                                               asset_server: Res<AssetServer>,
                                                                                                                               mut materials: ResMut<Assets<StandardMaterial>>) {
        let robot = &robot.0;
        RoboticsActions::action_spawn_robot_as_stl_meshes(robot, &mut commands, &*asset_server, &mut *materials, 0);
    }
    pub fn system_robot_state_updater<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static>(robot: Res<BevyORobot<T, C, L>>,
                                                                                                                   mut updater: ResMut<UpdaterRobotState>,
                                                                                                                   mut query: Query<(&LinkMeshID, &mut Transform)>) {
        while updater.robot_state_update_requests.len() > 0 {
            let robot = &robot.0;
            let request = updater.robot_state_update_requests.pop().unwrap();
            let request_state: Vec<T> = request.1.iter().map(|x| T::constant(*x)).collect();
            RoboticsActions::action_set_state_of_robot(robot, &request_state, request.0, &mut query);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait BevyRoboticsTrait {
    fn bevy_display(&self);
}

impl<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static> BevyRoboticsTrait for ORobot<T, C, L> {
    fn bevy_display(&self) {
        App::new()
            .optima_bevy_base()
            .optima_bevy_robotics_base(self.clone())
            .optima_bevy_pan_orbit_camera()
            .optima_bevy_starter_lights()
            .optima_bevy_spawn_robot::<T, C, L>()
            .optima_bevy_robotics_scene_visuals_starter()
            .run();
    }
}

impl<T: AD, C: O3DPoseCategoryTrait  + 'static, L: OLinalgCategoryTrait + 'static> BevyRoboticsTrait for OChain<T, C, L> {
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
pub struct UpdaterRobotState {
    pub (crate) robot_state_update_requests: Vec<(usize, Vec<f64>)>
}
impl UpdaterRobotState {
    pub fn new() -> Self {
        Self { robot_state_update_requests: vec![] }
    }
    pub fn add_update_request<T: AD, V: OVec<T>>(&mut self, robot_instance_idx: usize, state: &V) {
        let save_state = state.to_constant_vec();
        self.robot_state_update_requests.push( (robot_instance_idx, save_state) );
    }
}

#[derive(Resource)]
pub struct BevyORobot<T: AD, C: O3DPoseCategoryTrait + Send + 'static, L: OLinalgCategoryTrait>(pub ORobot<T, C, L>);
