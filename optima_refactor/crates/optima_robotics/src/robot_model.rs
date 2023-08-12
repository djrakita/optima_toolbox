use std::fmt::Debug;
use std::marker::PhantomData;
use urdf_rs::{Collision, Color, Dynamics, Geometry, Inertial, Joint, JointLimit, JointType, Link, Material, Mimic, Pose, SafetyController, Texture, Visual};
use serde::{Serialize, Deserialize};
use optima_utils::arr_storage::*;
use crate::utils::get_urdf_path_from_robot_name;

pub const MAX_NUM_COLLISION: usize = 2;
pub const MAX_NUM_VISUAL: usize = 2;
pub const MAX_LEN_NAME_STRINGS: usize = 65;
pub const MAX_LEN_FILE_STRINGS: usize = 100;
pub const MAX_NUM_LINK_CONVEX_SUBCOMPONENTS: usize = 6;

pub type ORobotModelDefault = ORobotModel<ArrayVecStor, 100, 100>;

/// Make sure nothing in ORobotModel is stateful
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ORobotModel<L: ArrStorage, const MAX_NUM_LINKS: usize, const MAX_NUM_JOINTS: usize> {
    #[serde(deserialize_with = "L::ArrType::deserialize")]
    links: L::ArrType<OLink<L>, MAX_NUM_LINKS>,
    #[serde(deserialize_with = "L::ArrType::deserialize")]
    joints: L::ArrType<OJoint<L>, MAX_NUM_JOINTS>
}
impl<L: ArrStorage, const MAX_NUM_LINKS: usize, const MAX_NUM_JOINTS: usize> ORobotModel<L, MAX_NUM_LINKS, MAX_NUM_JOINTS> {
    pub fn new_empty() -> Self {
        Self {
            links: Default::default(),
            joints: Default::default()
        }
    }
    pub fn add_from_urdf(&mut self, robot_name: &str) {
        let urdf_path = get_urdf_path_from_robot_name(robot_name);
        let urdf = urdf_path.load_urdf();

        urdf.links.iter().for_each(|x| {
            self.links.push(OLink::from_link(x) );
        });

        urdf.joints.iter().for_each(|x| {
            self.joints.push(OJoint::from_joint(x) );
        });
    }
    pub fn links(&self) -> &L::ArrType<OLink<L>, MAX_NUM_LINKS> {
        &self.links
    }
    pub fn joints(&self) -> &L::ArrType<OJoint<L>, MAX_NUM_JOINTS> {
        &self.joints
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OLink<L: ArrStorage> {
    link_idx: usize,
    #[serde(deserialize_with = "L::StrType::deserialize")]
    name: L::StrType<MAX_LEN_NAME_STRINGS>,
    #[serde(deserialize_with = "L::ArrType::deserialize")]
    collision: L::ArrType<OCollision<L>, MAX_NUM_COLLISION>,
    #[serde(deserialize_with = "L::ArrType::deserialize")]
    visual: L::ArrType<OVisual<L>, MAX_NUM_VISUAL>,
    inertial: OInertial
}
impl<L: ArrStorage> OLink<L> {
    fn from_link(link: &Link) -> Self {
        Self {
            link_idx: 0,
            name: L::StrType::from_str(&link.name),
            collision: link.collision.iter().map(|x| OCollision::from_collision(x) ).collect(),
            visual: link.visual.iter().map(|x| OVisual::from_visual(x) ).collect(),
            inertial: OInertial::from_inertial(&link.inertial)
        }
    }
    pub fn inertial(&self) -> &OInertial {
        &self.inertial
    }
    pub fn link_idx(&self) -> usize {
        self.link_idx
    }
    pub fn name(&self) -> &L::StrType<MAX_LEN_NAME_STRINGS> {
        &self.name
    }
    pub fn collision(&self) -> &L::ArrType<OCollision<L>, MAX_NUM_COLLISION> {
        &self.collision
    }
    pub fn visual(&self) -> &L::ArrType<OVisual<L>, MAX_NUM_VISUAL> {
        &self.visual
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OJoint<L: ArrStorage> {
    joint_idx: usize,
    #[serde(deserialize_with = "L::StrType::deserialize")]
    name: L::StrType<MAX_LEN_NAME_STRINGS>,
    joint_type: OJointType,
    origin: OPose,
    axis: [f64; 3],
    #[serde(deserialize_with = "L::StrType::deserialize")]
    parent_link: L::StrType<MAX_LEN_NAME_STRINGS>,
    #[serde(deserialize_with = "L::StrType::deserialize")]
    child_link: L::StrType<MAX_LEN_NAME_STRINGS>,
    limit: OJointLimit,
    dynamics: Option<ODynamics>,
    #[serde(deserialize_with = "Option::<OMimic<_>>::deserialize")]
    mimic: Option<OMimic<L>>,
    safety_controller: Option<OSafetyController>
}
impl<L: ArrStorage> OJoint<L> {
    fn from_joint(joint: &Joint) -> Self {
        Self {
            joint_idx: 0,
            name: L::StrType::from_str(&joint.name),
            joint_type: OJointType::from_joint_type(&joint.joint_type),
            origin: OPose::from_pose(&joint.origin),
            axis: joint.axis.xyz.0,
            parent_link: L::StrType::from_str(&joint.parent.link),
            child_link: L::StrType::from_str(&joint.child.link),
            limit: OJointLimit::from_joint_limit(&joint.limit),
            dynamics: match &joint.dynamics {
                None => { None }
                Some(dynamics) => { Some(ODynamics::from_dynamics(dynamics)) }
            },
            mimic: match &joint.mimic {
                None => { None }
                Some(mimic) => { Some(OMimic::from_mimic(mimic)) }
            },
            safety_controller: match &joint.safety_controller {
                None => { None }
                Some(safety_controller) => { Some(OSafetyController::from_safety_controller(safety_controller)) }
            }
        }
    }
    pub fn joint_idx(&self) -> usize {
        self.joint_idx
    }
    pub fn name(&self) -> &L::StrType<MAX_LEN_NAME_STRINGS> {
        &self.name
    }
    pub fn joint_type(&self) -> &OJointType {
        &self.joint_type
    }
    pub fn origin(&self) -> &OPose {
        &self.origin
    }
    pub fn axis(&self) -> &[f64; 3] {
        &self.axis
    }
    pub fn parent_link(&self) -> &L::StrType<MAX_LEN_NAME_STRINGS> {
        &self.parent_link
    }
    pub fn child_link(&self) -> &L::StrType<MAX_LEN_NAME_STRINGS> {
        &self.child_link
    }
    pub fn limit(&self) -> &OJointLimit {
        &self.limit
    }
    pub fn dynamics(&self) -> &Option<ODynamics> {
        &self.dynamics
    }
    pub fn mimic(&self) -> &Option<OMimic<L>> {
        &self.mimic
    }
    pub fn safety_controller(&self) -> &Option<OSafetyController> {
        &self.safety_controller
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OInertial {
    ixx: f64,
    ixy: f64,
    ixz: f64,
    iyy: f64,
    iyz: f64,
    izz: f64
}
impl OInertial {
    fn from_inertial(inertial: &Inertial) -> Self {
        Self {
            ixx: inertial.inertia.ixx,
            ixy: inertial.inertia.ixy,
            ixz: inertial.inertia.ixz,
            iyy: inertial.inertia.iyy,
            iyz: inertial.inertia.iyz,
            izz: inertial.inertia.izz
        }
    }
    pub fn ixx(&self) -> f64 {
        self.ixx
    }
    pub fn ixy(&self) -> f64 {
        self.ixy
    }
    pub fn ixz(&self) -> f64 {
        self.ixz
    }
    pub fn iyy(&self) -> f64 {
        self.iyy
    }
    pub fn iyz(&self) -> f64 {
        self.iyz
    }
    pub fn izz(&self) -> f64 {
        self.izz
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OCollision<L: ArrStorage> {
    #[serde(deserialize_with = "OGeometry::deserialize")]
    geometry: OGeometry<L>,
    #[serde(deserialize_with = "Option::<L::StrType<MAX_LEN_NAME_STRINGS>>::deserialize")]
    name: Option<L::StrType<MAX_LEN_NAME_STRINGS>>,
    origin: OPose
}
impl<L: ArrStorage> OCollision<L> {
    fn from_collision(collision: &Collision) -> Self {
        Self {
            geometry: OGeometry::from_geometry(&collision.geometry),
            name: match &collision.name {
                None => { None }
                Some(name) => { Some(L::StrType::from_str(name)) }
            },
            origin: OPose::from_pose(&collision.origin)
        }
    }
    pub fn geometry(&self) -> &OGeometry<L> {
        &self.geometry
    }
    pub fn name(&self) -> &Option<L::StrType<MAX_LEN_NAME_STRINGS>> {
        &self.name
    }
    pub fn origin(&self) -> &OPose {
        &self.origin
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OVisual<L: ArrStorage> {
    #[serde(deserialize_with = "Option::<L::StrType<MAX_LEN_NAME_STRINGS>>::deserialize")]
    name: Option<L::StrType<MAX_LEN_NAME_STRINGS>>,
    #[serde(deserialize_with = "Option::<OMaterial<L>>::deserialize")]
    material: Option<OMaterial<L>>,
    origin: OPose,
    #[serde(deserialize_with = "OGeometry::deserialize")]
    geometry: OGeometry<L>
}
impl<L: ArrStorage> OVisual<L> {
    fn from_visual(visual: &Visual) -> Self {
        Self {
            name: match &visual.name {
                None => { None }
                Some(name) => { Some(L::StrType::from_str(name)) }
            },
            material: match &visual.material {
                None => { None }
                Some(material) => { Some(OMaterial::from_material(material)) }
            },
            origin: OPose::from_pose(&visual.origin),
            geometry: OGeometry::from_geometry(&visual.geometry)
        }
    }
    pub fn name(&self) -> &Option<L::StrType<MAX_LEN_NAME_STRINGS>> {
        &self.name
    }
    pub fn material(&self) -> &Option<OMaterial<L>> {
        &self.material
    }
    pub fn origin(&self) -> &OPose {
        &self.origin
    }
    pub fn geometry(&self) -> &OGeometry<L> {
        &self.geometry
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OMaterial<L: ArrStorage> {
    #[serde(deserialize_with = "Option::<OTexture<L>>::deserialize")]
    texture: Option<OTexture<L>>,
    name: String,
    color: Option<OColor>,
    _phantom_data: PhantomData<L>
}
impl<L: ArrStorage> OMaterial<L> {
    fn from_material(material: &Material) -> Self {
        Self {
            texture: match &material.texture {
                None => { None }
                Some(texture) => { Some(OTexture::from_texture(texture)) }
            },
            name: material.name.clone(),
            color: match &material.color {
                None => { None }
                Some(color) => { Some(OColor::from_color(color)) }
            },
            _phantom_data: Default::default()
        }
    }
    pub fn texture(&self) -> &Option<OTexture<L>> {
        &self.texture
    }
    pub fn name(&self) -> &str {
        &self.name
    }
    pub fn color(&self) -> &Option<OColor> {
        &self.color
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OTexture<L: ArrStorage> {
    filename: L::StrType<MAX_LEN_FILE_STRINGS>,
    _phantom_data: PhantomData<L>
}
impl<L: ArrStorage> OTexture<L> {
    fn from_texture(texture: &Texture) -> Self {
        Self {
            filename: L::StrType::from_str(&texture.filename),
            _phantom_data: Default::default()
        }
    }
    pub fn filename(&self) -> &L::StrType<MAX_LEN_FILE_STRINGS> {
        &self.filename
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OColor {
    rgba: [f64; 4]
}
impl OColor {
    fn from_color(color: &Color) -> Self {
        Self {
            rgba: color.rgba.0
        }
    }
    pub fn rgba(&self) -> &[f64; 4] {
        &self.rgba
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum OGeometry<L: ArrStorage> {
    Box { size: [f64; 3] },
    Cylinder { radius: f64, length: f64 },
    Capsule { radius: f64, length: f64 },
    Sphere { radius: f64 },
    // #[serde(deserialize_with = "L::StrType::deserialize")]
    Mesh { filename: L::StrType<MAX_LEN_FILE_STRINGS>, scale: Option<[f64; 3]> }
}
impl<L: ArrStorage> OGeometry<L> {
    fn from_geometry(geometry: &Geometry) -> Self {
        return match &geometry {
            Geometry::Box { size } => { OGeometry::Box { size: size.0 } }
            Geometry::Cylinder { radius, length } => { OGeometry::Cylinder { radius: *radius, length: *length } }
            Geometry::Capsule { radius, length } => { OGeometry::Capsule { radius: *radius, length: *length } }
            Geometry::Sphere { radius } => { OGeometry::Sphere { radius: *radius } }
            Geometry::Mesh { filename, scale } => { OGeometry::Mesh { filename: L::StrType::from_str(filename), scale: match scale { None => { None } Some(scale) => { Some(scale.0) } } } }
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OPose {
    rpy: [f64; 3],
    xyz: [f64; 3]
}
impl OPose {
    fn from_pose(pose: &Pose) -> Self {
        Self {
            rpy: pose.rpy.0,
            xyz: pose.xyz.0
        }
    }
    pub fn rpy(&self) -> &[f64; 3] {
        &self.rpy
    }
    pub fn xyz(&self) -> &[f64; 3] {
        &self.xyz
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum OJointType {
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar,
    Spherical
}
impl OJointType {
    fn from_joint_type(joint_type: &JointType) -> Self {
        match joint_type {
            JointType::Revolute => { OJointType::Revolute }
            JointType::Continuous => { OJointType::Continuous }
            JointType::Prismatic => { OJointType::Prismatic }
            JointType::Fixed => { OJointType::Fixed }
            JointType::Floating => { OJointType::Floating }
            JointType::Planar => { OJointType::Planar }
            JointType::Spherical => { OJointType::Spherical }
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OMimic<L: ArrStorage> {
    joint: L::StrType<MAX_LEN_NAME_STRINGS>,
    multiplier: Option<f64>,
    offset: Option<f64>
}
impl<L: ArrStorage> OMimic<L> {
    fn from_mimic(mimic: &Mimic) -> Self {
        Self {
            joint: L::StrType::from_str(&mimic.joint),
            multiplier: mimic.multiplier.clone(),
            offset: mimic.offset.clone()
        }
    }
    pub fn multiplier(&self) -> &Option<f64> {
        &self.multiplier
    }
    pub fn offset(&self) -> &Option<f64> {
        &self.offset
    }
    pub fn joint(&self) -> &L::StrType<MAX_LEN_NAME_STRINGS> {
        &self.joint
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ODynamics {
    damping: f64,
    friction: f64
}
impl ODynamics {
    fn from_dynamics(dynamics: &Dynamics) -> Self {
        Self {
            damping: dynamics.damping,
            friction: dynamics.friction
        }
    }
    pub fn damping(&self) -> f64 {
        self.damping
    }
    pub fn friction(&self) -> f64 {
        self.friction
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OJointLimit {
    effort: f64,
    lower: f64,
    upper: f64,
    velocity: f64
}
impl OJointLimit {
    fn from_joint_limit(joint_limit: &JointLimit) -> Self {
        Self {
            effort: joint_limit.effort,
            lower: joint_limit.lower,
            upper: joint_limit.upper,
            velocity: joint_limit.velocity
        }
    }
    pub fn effort(&self) -> f64 {
        self.effort
    }
    pub fn lower(&self) -> f64 {
        self.lower
    }
    pub fn upper(&self) -> f64 {
        self.upper
    }
    pub fn velocity(&self) -> f64 {
        self.velocity
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OSafetyController {
    k_position: f64,
    k_velocity: f64,
    soft_lower_limit: f64,
    soft_upper_limit: f64
}
impl OSafetyController {
    fn from_safety_controller(safety_controller: &SafetyController) -> Self {
        Self {
            k_position: safety_controller.k_position,
            k_velocity: safety_controller.k_velocity,
            soft_lower_limit: safety_controller.soft_lower_limit,
            soft_upper_limit: safety_controller.soft_upper_limit
        }
    }
    pub fn k_position(&self) -> f64 {
        self.k_position
    }
    pub fn k_velocity(&self) -> f64 {
        self.k_velocity
    }
    pub fn soft_lower_limit(&self) -> f64 {
        self.soft_lower_limit
    }
    pub fn soft_upper_limit(&self) -> f64 {
        self.soft_upper_limit
    }
}

