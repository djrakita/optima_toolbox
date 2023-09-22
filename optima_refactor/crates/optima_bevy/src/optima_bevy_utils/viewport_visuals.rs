use bevy::asset::{Assets};
use bevy::math::{Mat3, Quat, Vec3};
use bevy::pbr::PbrBundle;
use bevy::prelude::{Color, Commands, Entity, Gizmos, Mesh, ResMut, shape, StandardMaterial, Transform};
use nalgebra::DVector;
use optima_geometry::get_points_around_circle;

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
    pub fn action_draw_gpu_line_optima_space(gizmos: &mut Gizmos,
                                             start_point: Vec3,
                                             end_point: Vec3,
                                             color: Color,
                                             width_in_mm: f32,
                                             num_points_per_circle: usize,
                                             num_concentric_circles: usize) {
        let new_start_point = Vec3::new(start_point[0], start_point[2], -start_point[1]);
        let new_end_point = Vec3::new(end_point[0], end_point[2], -end_point[1]);

        Self::action_draw_gpu_line_bevy_space2(gizmos, new_start_point, new_end_point, color, width_in_mm, num_points_per_circle, num_concentric_circles);
    }
    pub fn action_draw_gpu_line_bevy_space2(gizmos: &mut Gizmos,
                                           start_point: Vec3,
                                           end_point: Vec3,
                                           color: Color,
                                           width_in_mm: f32,
                                           num_points_per_circle: usize,
                                           num_concentric_circles: usize) {
        assert!(width_in_mm >= 0.0);
        assert!(num_concentric_circles >= 1);
        assert!(num_points_per_circle > 2);

        let start_point_dvec = DVector::from_vec(vec![start_point.x as f64, start_point.y as f64, start_point.z as f64]);
        let end_point_dvec = DVector::from_vec(vec![end_point.x as f64, end_point.y as f64, end_point.z as f64]);

        let circle = get_points_around_circle(&DVector::from_vec(vec![0., 0., 0.]), &(&end_point_dvec - &start_point_dvec), 1.0, num_points_per_circle, Some(0));

        if width_in_mm > 0.0 {
            let width_stride = width_in_mm / num_concentric_circles as f32;
            let mut widths = vec![];
            for i in 0..num_concentric_circles {
                widths.push((width_in_mm - i as f32 * width_stride) / 1000.0);
            }

            for width in widths {
                let num_circle_points = circle.len();
                for i in 0..num_circle_points-1 {
                    let scaled_curr_circle_point = width as f64 * circle[i].clone();
                    let scaled_next_circle_point = width as f64 * circle[i+1].clone();
                    let scaled_curr_circle_point_as_vec3 = Vec3::new(scaled_curr_circle_point[0] as f32, scaled_curr_circle_point[1] as f32, scaled_curr_circle_point[2] as f32);
                    let scaled_next_circle_point_as_vec3 = Vec3::new(scaled_next_circle_point[0] as f32, scaled_next_circle_point[1] as f32, scaled_next_circle_point[2] as f32);

                    gizmos.line(scaled_curr_circle_point_as_vec3 + start_point, scaled_next_circle_point_as_vec3 + start_point, color.clone());
                    gizmos.line(scaled_curr_circle_point_as_vec3 + end_point, scaled_next_circle_point_as_vec3 + end_point, color.clone());
                }

                for circle_point in &circle {
                    let scaled_circle_point = width as f64 * circle_point;
                    let scaled_circle_point_as_vec3 = Vec3::new(scaled_circle_point[0] as f32, scaled_circle_point[1] as f32, scaled_circle_point[2] as f32);
                    gizmos.line(scaled_circle_point_as_vec3 + start_point, scaled_circle_point_as_vec3 + end_point, color.clone());
                }
            }
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