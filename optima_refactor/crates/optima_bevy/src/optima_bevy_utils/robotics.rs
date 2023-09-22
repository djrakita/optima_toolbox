use ad_trait::AD;
use bevy::pbr::StandardMaterial;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::egui::panel::Side;
use bevy_egui::egui::Ui;
use bevy_egui::{egui, EguiContexts};
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategoryTrait};
use optima_bevy_egui::{OEguiContainerTrait, OEguiEngineWrapper, OEguiSidePanel, OEguiSlider, OEguiWidgetTrait};
use optima_linalg::{OLinalgCategoryTrait, OVec};
use optima_robotics::chain::{ChainFKResult, OChain};
use optima_robotics::robot::ORobot;
use optima_robotics::robotics_traits::AsChainTrait;
use crate::optima_bevy_utils::file::get_asset_path_str_from_ostemcellpath;
use crate::optima_bevy_utils::transform::TransformUtils;
use crate::{BevySystemSet, OptimaBevyTrait};

pub struct RoboticsActions;
impl RoboticsActions {
    pub fn action_spawn_chain_as_stl_meshes<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait>(chain: &OChain<T, C, L>,
                                                                                                     fk_res: &ChainFKResult<T, C::P<T>>,
                                                                                                     commands: &mut Commands,
                                                                                                     asset_server: &AssetServer,
                                                                                                     materials: &mut Assets<StandardMaterial>,
                                                                                                     chain_instance_idx: usize) {
        chain.links().iter().enumerate().for_each(|(link_idx, link)| {
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
                            chain_instance_idx,
                            sub_chain_idx: link.sub_chain_idx(),
                            link_idx,
                        });
                    }
                }
            }
        });
    }
    pub fn action_set_state_of_chain<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait, V: OVec<T>>(chain: &OChain<T, C, L>,
                                                                                                          state: &V,
                                                                                                          chain_instance_idx: usize,
                                                                                                          query: &mut Query<(&LinkMeshID, &mut Transform)>) {
        let fk_res = chain.forward_kinematics(state, None);
        for (link_mesh_id, mut transform) in query.iter_mut() {
            let link_mesh_id: &LinkMeshID = &link_mesh_id;
            let transform: &mut Transform = &mut transform;

            if link_mesh_id.chain_instance_idx == chain_instance_idx {
                let link_idx = link_mesh_id.link_idx;
                let link = &chain.links()[link_idx];
                let pose = fk_res.get_link_pose(link_idx).as_ref().unwrap();
                let visual_offset = link.visual()[0].origin().pose();
                *transform = TransformUtils::util_convert_3d_pose_to_y_up_bevy_transform(&(pose.mul(visual_offset)));
            }
        }
    }
    pub fn action_chain_joint_sliders_egui<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait>(chain: &OChain<T, C, L>,
                                                                                                    updater: &mut ResMut<UpdaterChainState>,
                                                                                                    egui_engine: &Res<OEguiEngineWrapper>,
                                                                                                    ui: &mut Ui) {
        let mut reset_clicked = false;
        ui.horizontal(|ui| {
            ui.heading("Joint Sliders");
            reset_clicked = ui.button("Reset").clicked();
        });
        ui.group(|ui| {
            egui::ScrollArea::new([true, true])
                .max_height(500.)
                .show(ui, |ui| {
                    chain.joints().iter().for_each(|joint| {
                        let dof_idxs = joint.dof_idxs();
                        for (i, dof_idx) in dof_idxs.iter().enumerate() {
                            let label = format!("joint_slider_dof_{}", dof_idx);
                            let lower = joint.limit().lower()[i];
                            let upper = joint.limit().upper()[i];

                            ui.separator();
                            ui.label(format!("DOF idx {}", dof_idx));
                            ui.label(format!("{}, sub dof {}", joint.name(), i));
                            ui.label(format!("Joint type {:?}, Axis {:?}", joint.joint_type(), joint.axis()));
                            OEguiSlider::new(lower.to_constant(), upper.to_constant())
                                .show(&label, ui, &egui_engine, &());

                            let mut mutex_guard = egui_engine.get_mutex_guard();
                            let response = mutex_guard.get_slider_response_mut(&label).expect("error");

                            ui.horizontal(|ui| {
                                if ui.button("0.0").clicked() { response.slider_value = 0.0; }
                                if ui.button("+0.01").clicked() { response.slider_value += 0.01; }
                                if ui.button("-0.01").clicked() { response.slider_value -= 0.01; }
                                if ui.button("+0.1").clicked() { response.slider_value += 0.1; }
                                if ui.button("-0.1").clicked() { response.slider_value -= 0.1; }
                            });
                        }
                    });
                });
        });

        let mut mutex_guard = egui_engine.get_mutex_guard();

        let num_dofs = chain.num_dofs();
        let mut curr_state = vec![T::zero(); chain.num_dofs()];
        for i in 0..num_dofs {
            let label = format!("joint_slider_dof_{}", i);
            let response = mutex_guard.get_slider_response_mut(&label).expect("error");
            if reset_clicked { response.slider_value = 0.0; }
            let value = response.slider_value();
            curr_state[i] = T::constant(value);
        }

        updater.add_update_request(0, &OVec::to_other_ad_type::<T>(&curr_state));
    }
    pub fn action_chain_link_vis_panel_egui<T: AD, C: O3DPoseCategoryTrait, L: OLinalgCategoryTrait>(_chain: &OChain<T, C, L>,
                                                                                                     _egui_engine: &Res<OEguiEngineWrapper>,
                                                                                                     _ui: &mut Ui) {

    }
}

pub struct RoboticsSystems;
impl RoboticsSystems {
    pub fn system_spawn_chain_links_as_stl_meshes<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static>(chain: Res<BevyOChain<T, C, L>>,
                                                                                                                               mut commands: Commands,
                                                                                                                               asset_server: Res<AssetServer>,
                                                                                                                               mut materials: ResMut<Assets<StandardMaterial>>) {
        let chain = &chain.0;
        let num_dofs = chain.num_dofs();
        let fk_res = chain.forward_kinematics(&vec![T::zero(); num_dofs], None);
        RoboticsActions::action_spawn_chain_as_stl_meshes(chain, &fk_res, &mut commands, &*asset_server, &mut *materials, 0);
    }
    pub fn system_chain_state_updater<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static>(chain: Res<BevyOChain<T, C, L>>,
                                                                                                                   mut updater: ResMut<UpdaterChainState>,
                                                                                                                   mut query: Query<(&LinkMeshID, &mut Transform)>) {
        while updater.chain_state_update_requests.len() > 0 {
            let chain = &chain.0;
            let request = updater.chain_state_update_requests.pop().unwrap();
            let request_state: Vec<T> = request.1.iter().map(|x| T::constant(*x)).collect();
            RoboticsActions::action_set_state_of_chain(chain, &request_state, request.0, &mut query);
        }
    }
    pub fn system_chain_main_info_panel_egui<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static>(chain: Res<BevyOChain<T, C, L>>,
                                                                                                                          mut contexts: EguiContexts,
                                                                                                                          mut updater: ResMut<UpdaterChainState>,
                                                                                                                          egui_engine: Res<OEguiEngineWrapper>,
                                                                                                                          window_query: Query<&Window, With<PrimaryWindow>>) {
        OEguiSidePanel::new(Side::Left, 300.0)
            .show("joint_sliders_side_panel", contexts.ctx_mut(), &egui_engine, &window_query, &(), |ui| {
                RoboticsActions::action_chain_joint_sliders_egui(&chain.0, &mut updater, &egui_engine, ui);
            });
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait BevyRoboticsTrait {
    fn bevy_display(&self);
}

impl<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static> BevyRoboticsTrait for OChain<T, C, L> {
    fn bevy_display(&self) {

        App::new()
            .optima_bevy_base()
            .optima_bevy_robotics_base(self.clone())
            .optima_bevy_pan_orbit_camera()
            .optima_bevy_starter_lights()
            .optima_bevy_spawn_chain::<T, C, L>()
            .optima_bevy_robotics_scene_visuals_starter()
            .optima_bevy_egui()
            .add_systems(Update, RoboticsSystems::system_chain_main_info_panel_egui::<T, C, L>.before(BevySystemSet::Camera))
            .run();
    }
}
impl<T: AD, C: O3DPoseCategoryTrait + 'static, L: OLinalgCategoryTrait + 'static> BevyRoboticsTrait for ORobot<T, C, L> {
    fn bevy_display(&self) {
        self.as_chain().bevy_display();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Component)]
pub struct LinkMeshID {
    pub chain_instance_idx: usize,
    pub sub_chain_idx: usize,
    pub link_idx: usize
}

#[derive(Resource)]
pub struct UpdaterChainState {
    pub (crate) chain_state_update_requests: Vec<(usize, Vec<f64>)>
}
impl UpdaterChainState {
    pub fn new() -> Self {
        Self { chain_state_update_requests: vec![] }
    }
    pub fn add_update_request<T: AD, V: OVec<T>>(&mut self, chain_instance_idx: usize, state: &V) {
        let save_state = state.to_constant_vec();
        self.chain_state_update_requests.push( (chain_instance_idx, save_state) );
    }
}

#[derive(Resource)]
pub struct BevyOChain<T: AD, C: O3DPoseCategoryTrait + Send + 'static, L: OLinalgCategoryTrait>(pub OChain<T, C, L>);
