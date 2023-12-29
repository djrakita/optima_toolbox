use nalgebra::{Isometry3, Vector3};
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_3d_spatial::optima_3d_rotation::QuatConstructor;
use optima_bevy::{App, OptimaBevyTrait};
use optima_bevy::optima_bevy_utils::viewport_visuals::BevyDrawShape;
use optima_proximity::parry_ad::shape::Cuboid;
use optima_proximity::shape_scene::OParryGenericShapeScene;
use optima_proximity::shapes::OParryShape;
use optima_robotics::robot::ORobotDefault;

fn main() {
    let shapes = vec![
        OParryShape::new_default(Cuboid::new(Vector3::new(0.029816824170463455 * 0.5,1.4124879353697206 * 0.5,0.7242890356101396 * 0.5)), Isometry3::identity()),
        OParryShape::new_default(Cuboid::new(Vector3::new(0.5483168931687388 * 0.5,1.4124879353697206 * 0.5,0.029816824170463455 * 0.5)), Isometry3::identity()),
        OParryShape::new_default(Cuboid::new(Vector3::new(0.5483168931687388 * 0.5,0.029816824170463455 * 0.5,0.6159011632941881 * 0.5)), Isometry3::identity()),
        OParryShape::new_default(Cuboid::new(Vector3::new(0.5483168931687388 * 0.5,0.029816824170463455 * 0.5,0.6159011632941881 * 0.5)), Isometry3::identity()),
        OParryShape::new_default(Cuboid::new(Vector3::new(0.5483168931687388 * 0.5,0.029816824170463455 * 0.5,0.6159011632941881 * 0.5)), Isometry3::identity()),
        OParryShape::new_default(Cuboid::new(Vector3::new(0.5483168931687388 * 0.5,1.4124879353697206 * 0.5,0.029816824170463455 * 0.5)), Isometry3::identity()),
        OParryShape::new_default(Cuboid::new(Vector3::new(0.3 * 0.5,0.25 * 0.5,0.8 * 0.5)), Isometry3::identity())
    ];

    let poses = vec![
        Isometry3::from_constructors(&[1.190814282258407, -0.13550199346642616, 0.3621445178050698], &QuatConstructor {
            w: 0.9966893968483386,
            x: 0.0,
            y: 0.0,
            z: -0.0813034206543318,
        }),
        Isometry3::from_constructors(&[0.9202803453390374, -0.09106954126692335, 0.13820469648641495], &QuatConstructor {
            w: 0.9966893968483386,
            x: 0.0,
            y: 0.0,
            z: -0.0813034206543318,
        }),
        Isometry3::from_constructors(&[0.8058204345600789, -0.7879766159302007, 0.4312468660482773], &QuatConstructor {
            w: 0.9966893968483386,
            x: 0.0,
            y: 0.0,
            z: -0.0813034206543318,
        }),
        Isometry3::from_constructors(&[1.034740256117996, 0.6058375333963542, 0.4312468660482773], &QuatConstructor {
            w: 0.9966893968483386,
            x: 0.0,
            y: 0.0,
            z: -0.0813034206543318,
        }),
        Isometry3::from_constructors(&[0.927160824683477, -0.04917666628928477, 0.4312468660482773], &QuatConstructor {
            w: 0.9966893968483386,
            x: 0.0,
            y: 0.0,
            z: -0.0813034206543318,
        }),
        Isometry3::from_constructors(&[0.927160824683477, -0.09106954126692335, 0.41456459296377046], &QuatConstructor {
            w: 0.9966893968483386,
            x: 0.0,
            y: 0.0,
            z: -0.0813034206543318,
        }),
        Isometry3::from_constructors(&[-0.05, 0.0, -0.4], &QuatConstructor {
            w: 1.0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }),
    ];

    let scene = OParryGenericShapeScene::new(shapes, poses);
    let robot = ORobotDefault::from_urdf("panda");

    let mut app = App::new();
    app.optima_bevy_starter_scene();
    app.optima_bevy_spawn_generic_shape_scene(scene.clone());
    app.optima_bevy_spawn_robot_in_pose(robot.clone(), vec![1.3835341759012247, 0.644993809355163, -1.231244289043802, -1.599375351855726, -2.708679434572325, 2.9128314876962325, -2.173304133837488, 0.0], 0);
    app.optima_bevy_draw_shape(BevyDrawShape::Sphere { radius: 0.01 }, Isometry3::from_constructors(&[0.6539468661968684, 0.2720397734086678, 0.3551723678894053], &[0.,0.,0.]));

    app.run();
}