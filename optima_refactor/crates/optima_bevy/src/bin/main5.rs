use bevy::app::{App, Startup};
use bevy::asset::{Assets, AssetServer};
use bevy::pbr::StandardMaterial;
use bevy::prelude::{Commands, Mesh, Res, ResMut};
use nalgebra::{Isometry3, Vector3};
use parry_ad::shape::Cuboid;
use optima_bevy::optima_bevy_utils::shape_scene::{ShapeSceneActions, ShapeSceneType};
use optima_bevy::OptimaBevyTrait;
use optima_proximity::shape_scene::OParryGenericShapeScene;
use optima_proximity::shapes::OParryShape;
use optima_robotics::robot::ORobotDefault;

fn main() {
    let r = ORobotDefault::from_urdf("panda");
    // let g = OParryGenericShapeScene::new(vec![OParryShape::new(Cuboid::new(Vector3::new(0.1,0.1,0.01)), Isometry3::identity())], vec![Isometry3::identity()]);

    let mut app = App::new();
    app.optima_bevy_starter_scene();
    app.add_systems(Startup, move |mut commands: Commands, asset_server: Res<AssetServer>, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>| {
        ShapeSceneActions::action_spawn_shape_scene::<_, _, Vec<_>, _>(&r, &vec![0.0; 16], ShapeSceneType::Robot, &mut commands, &asset_server, &mut meshes, &mut materials);
    });
    app.run();
}