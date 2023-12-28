use std::borrow::Cow;
use ad_trait::AD;
use bevy::asset::AssetServer;
use bevy::pbr::{AlphaMode, PbrBundle};
use bevy::prelude::{Assets, Color, Commands, Component, Mesh, Res, ResMut, Resource, shape, StandardMaterial, Visibility};
use bevy::utils::default;
use parry_ad::shape::TypedShape;
use optima_3d_spatial::optima_3d_pose::{O3DPose, O3DPoseCategory};
use optima_proximity::shape_scene::{OParryGenericShapeScene, ShapeSceneTrait};
use optima_proximity::shapes::{OParryShape, OParryShpGeneric, OParryShpTrait};
use crate::optima_bevy_utils::file::get_asset_path_str_from_ostemcellpath;
use crate::optima_bevy_utils::transform::TransformUtils;

pub struct ShapeSceneActions;
impl ShapeSceneActions {
    pub fn action_spawn_shape_scene<'a, T: AD, P: O3DPose<T>, S: ShapeSceneTrait<T, P, ShapeType=OParryShape<T, P>>>(scene: &'a S,
                                                                                                                     input: S::GetPosesInput,
                                                                                                                     scene_type: ShapeSceneType,
                                                                                                                     commands: &mut Commands,
                                                                                                                     asset_server: &Res<AssetServer>,
                                                                                                                     meshes: &mut ResMut<Assets<Mesh>>,
                                                                                                                     materials: &mut ResMut<Assets<StandardMaterial>>) {
        let shapes = scene.get_shapes();
        let poses = scene.get_shape_poses(&input);

        for (i, parry_shape) in shapes.iter().enumerate() {
            let pose = &poses.as_ref()[i];
            let base_shape = parry_shape.base_shape();

            let bounding_sphere = base_shape.bounding_sphere();
            let obb = base_shape.obb();
            let full = base_shape.base_shape();

            Self::action_spawn_parry_shape_generic(&bounding_sphere, pose, ParryShapeSceneMeshLabel::new(scene_type, ShapeType::BoundingSphere, i), Visibility::Hidden, commands, asset_server, meshes, materials);
            Self::action_spawn_parry_shape_generic(&obb, pose, ParryShapeSceneMeshLabel::new(scene_type, ShapeType::OBB, i), Visibility::Hidden, commands, asset_server, meshes, materials);
            Self::action_spawn_parry_shape_generic(&full, pose, ParryShapeSceneMeshLabel::new(scene_type, ShapeType::ConvexShape, i), Visibility::Hidden, commands, asset_server, meshes, materials);

            let convex_subcomponents = parry_shape.convex_subcomponents();
            for convex_subcomponent in convex_subcomponents {
                let bounding_sphere = convex_subcomponent.bounding_sphere();
                let obb = convex_subcomponent.obb();
                let full = convex_subcomponent.base_shape();

                Self::action_spawn_parry_shape_generic(&bounding_sphere, pose, ParryShapeSceneMeshLabel::new(scene_type, ShapeType::SubcomponentsBoundingSphere, i), Visibility::Hidden, commands, asset_server, meshes, materials);
                Self::action_spawn_parry_shape_generic(&obb, pose, ParryShapeSceneMeshLabel::new(scene_type, ShapeType::SubcomponentsOBB, i), Visibility::Hidden, commands, asset_server, meshes, materials);
                Self::action_spawn_parry_shape_generic(&full, pose, ParryShapeSceneMeshLabel::new(scene_type, ShapeType::SubcomponentsConvexShape, i), Visibility::Visible, commands, asset_server, meshes, materials);
            }
        }
    }

    fn action_spawn_parry_shape_generic<T: AD, P: O3DPose<T>>(shape: &OParryShpGeneric<T, P>,
                                                              pose: &P,
                                                              label: ParryShapeSceneMeshLabel,
                                                              initial_visibility: Visibility,
                                                              commands: &mut Commands,
                                                              asset_server: &Res<AssetServer>,
                                                              meshes: &mut ResMut<Assets<Mesh>>,
                                                              materials: &mut ResMut<Assets<StandardMaterial>>) {
        let material = materials.add(StandardMaterial {
            base_color: Color::Rgba {
                red: 0.0,
                green: 0.6,
                blue: 1.0,
                alpha: 0.5,
            },
            alpha_mode: AlphaMode::Blend,
            ..default()
        });
        let new_pose = shape.get_isometry3_cow(pose);
        let transform = TransformUtils::util_convert_3d_pose_to_y_up_bevy_transform(new_pose.as_ref());

        let typed_shape = shape.boxed_shape().shape().as_typed_shape();
        let mesh = match typed_shape {
            TypedShape::Ball(ball) => {
                meshes.add(shape::UVSphere {
                    radius: ball.radius.to_constant() as f32,
                    sectors: 30,
                    stacks: 30,
                }.into())
            }
            TypedShape::Cuboid(c) => {
                meshes.add(shape::Box::new(c.half_extents[0].to_constant() as f32 * 2.0, c.half_extents[1].to_constant() as f32 * 2.0, c.half_extents[2].to_constant() as f32 * 2.0).into())
            }
            TypedShape::ConvexPolyhedron(_) => {
                let path = shape.boxed_shape().path().as_ref().expect("error");
                let asset_path_str = get_asset_path_str_from_ostemcellpath(&path);
                let h = asset_server.load(&asset_path_str);
                h
            }
            TypedShape::Cylinder(c) => {
                meshes.add(shape::Cylinder {
                    radius: c.radius.to_constant() as f32,
                    height: c.half_height.to_constant() as f32 * 2.0,
                    resolution: 12,
                    segments: 30,
                }.into())
            }
            _ => { return; }
        };

        commands.spawn(PbrBundle {
            mesh,
            material,
            transform,
            visibility: initial_visibility,
            ..default()
        }).insert(label);
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Component)]
pub struct ParryShapeSceneMeshLabel {
    pub scene_type: ShapeSceneType,
    pub shape_type: ShapeType,
    pub shape_idx: usize,
}
impl ParryShapeSceneMeshLabel {
    pub fn new(scene_type: ShapeSceneType, shape_type: ShapeType, shape_idx: usize) -> Self {
        Self { scene_type, shape_type, shape_idx }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Copy)]
pub enum ShapeType {
    BoundingSphere, OBB, ConvexShape, SubcomponentsBoundingSphere, SubcomponentsOBB, SubcomponentsConvexShape
}

#[derive(Clone, Debug, PartialEq, Eq, Copy)]
pub enum ShapeSceneType {
    Robot, Environment
}

pub struct ShapeSceneSystems;
impl ShapeSceneSystems {
    /*
    pub fn system_spawn_shape_scene<T: AD, P: O3DPose<T>, S: ShapeSceneTrait<T, P>>(shape_scene: Res<ShapeSceneBevyWrapper<T, P, S>>) {

    }
    */
}

#[derive(Resource)]
pub struct BevyOParryGenericShapeScene<T: AD, C: O3DPoseCategory>(pub OParryGenericShapeScene<T, C::P<T>>);
impl<T: AD, C: O3DPoseCategory> ShapeSceneTrait<T, C::P<T>> for BevyOParryGenericShapeScene<T, C> {
    type ShapeType = OParryShape<T, C::P<T>>;
    type GetPosesInput = ();
    type PairSkipsType = ();

    fn get_shapes(&self) -> &Vec<Self::ShapeType> {
        self.0.get_shapes()
    }

    fn get_shape_poses<'a>(&'a self, input: &'a Self::GetPosesInput) -> Cow<'a, Vec<C::P<T>>> {
        self.0.get_shape_poses(input)
    }

    fn sample_pseudorandom_input(&self) -> Self::GetPosesInput {
        ()
    }

    fn get_pair_skips(&self) -> &Self::PairSkipsType {
        self.0.get_pair_skips()
    }

    fn shape_id_to_shape_str(&self, id: u64) -> String {
        self.0.shape_id_to_shape_str(id)
    }
}