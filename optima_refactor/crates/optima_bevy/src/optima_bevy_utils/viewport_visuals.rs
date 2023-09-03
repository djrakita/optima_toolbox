use bevy::asset::{Assets};
use bevy::math::{Mat3, Quat, Vec3};
use bevy::pbr::PbrBundle;
use bevy::prelude::{Color, Commands, Entity, Mesh, ResMut, shape, StandardMaterial, Transform};

pub struct ViewportVisualsActions;
impl ViewportVisualsActions {
    pub fn action_spawn_line_bevy_space(commands: &mut Commands,
                                        meshes: &mut ResMut<Assets<Mesh>>,
                                        materials: &mut ResMut<Assets<StandardMaterial>>,
                                        start_point: Vec3,
                                        end_point: Vec3,
                                        color: Color,
                                        width_in_mm: f32,
                                        unlit: bool) -> Entity {
        let length = (end_point - start_point).length();
        let mesh = Mesh::from(shape::Capsule {
            radius: 0.001,
            rings: 6,
            depth: length,
            latitudes: 6,
            longitudes: 6,
            uv_profile: Default::default(),
        });

        let mat = StandardMaterial {
            base_color: color,
            unlit,
            ..Default::default()
        };

        let midpoint = (end_point + start_point) / 2.0;
        let mut y_axis = (end_point - start_point) / length;
        y_axis = y_axis.normalize();
        let d = y_axis.dot(Vec3::new(0., -1., 0.)).abs();
        let mut x_axis = if d < 0.99 { y_axis.cross(Vec3::new(0., -1., 0.)) } else { y_axis.cross(Vec3::new(0., 0., -1.)) };
        x_axis = x_axis.normalize();
        let mut z_axis = x_axis.cross(y_axis);
        z_axis = z_axis.normalize();
        let mat3 = Mat3::from_cols_array_2d(&[[x_axis.x, x_axis.y, x_axis.z], [y_axis.x, y_axis.y, y_axis.z], [z_axis.x, z_axis.y, z_axis.z]]);

        let mut t = Transform::default();
        t = t.with_scale(Vec3::new(width_in_mm, 1.0, width_in_mm));
        t.rotate(Quat::from_mat3(&mat3));
        t.translation = midpoint;

        let entity = commands.spawn(PbrBundle {
            mesh: meshes.add(mesh),
            material: materials.add(mat),
            transform: t.clone(),
            ..Default::default()
        });

        return entity.id();
    }
    pub fn action_spawn_line_optima_space(commands: &mut Commands,
                                          meshes: &mut ResMut<Assets<Mesh>>,
                                          materials: &mut ResMut<Assets<StandardMaterial>>,
                                          start_point: Vec3,
                                          end_point: Vec3,
                                          color: Color,
                                          width_in_mm: f32,
                                          unlit: bool) -> Entity {
        let new_start_point = Vec3::new(start_point[0], start_point[2], -start_point[1]);
        let new_end_point = Vec3::new(end_point[0], end_point[2], -end_point[1]);

        Self::action_spawn_line_bevy_space(commands, meshes, materials, new_start_point, new_end_point, color, width_in_mm, unlit)
    }
    pub fn action_draw_robotics_grid(commands: &mut Commands,
                                     meshes: &mut ResMut<Assets<Mesh>>,
                                     materials: &mut ResMut<Assets<StandardMaterial>>) {
        let x_and_y_width = 5.0;
        let normal_width = 2.0;
        let normal_color = Color::rgba(0.6,0.6,0.6,1.);

        Self::action_spawn_line_optima_space(commands, meshes, materials, Vec3::new(0., 0., 0.), Vec3::new(10., 0., 0.), Color::rgba(1.,0.,0.,1.), x_and_y_width, true);
        Self::action_spawn_line_optima_space(commands, meshes, materials, Vec3::new(0., 0., 0.), Vec3::new(-10., 0., 0.), normal_color, normal_width, true);

        Self::action_spawn_line_optima_space(commands, meshes, materials, Vec3::new(0., 0., 0.), Vec3::new(0., 10., 0.), Color::rgba(0.,1.,0.,1.), x_and_y_width, true);
        Self::action_spawn_line_optima_space(commands, meshes, materials, Vec3::new(0., 0., 0.), Vec3::new(0., -10., 0.), normal_color.clone(), normal_width, true);

        for i in 0..10 {
            Self::action_spawn_line_optima_space(commands, meshes, materials, Vec3::new(i as f32, -10.0, 0.), Vec3::new(i as f32, 10.0, 0.), normal_color.clone(), normal_width, true);
            Self::action_spawn_line_optima_space(commands, meshes, materials, Vec3::new(-i as f32, -10.0, 0.), Vec3::new(-i as f32, 10.0, 0.), normal_color.clone(), normal_width, true);

            Self::action_spawn_line_optima_space(commands, meshes, materials, Vec3::new(-10.0, i as f32, 0.), Vec3::new( 10.0, i as f32,0.), normal_color.clone(), normal_width, true);
            Self::action_spawn_line_optima_space(commands, meshes, materials, Vec3::new( -10.0, -i as f32,0.), Vec3::new(10.0, -i as f32, 0.), normal_color.clone(), normal_width, true);
        }
    }
}

pub struct ViewportVisualsSystems;
impl ViewportVisualsSystems {
    pub fn system_draw_robotics_grid(mut commands: Commands,
                                     mut meshes: ResMut<Assets<Mesh>>,
                                     mut materials: ResMut<Assets<StandardMaterial>>) {
        ViewportVisualsActions::action_draw_robotics_grid(&mut commands, &mut meshes, &mut materials);
    }
}