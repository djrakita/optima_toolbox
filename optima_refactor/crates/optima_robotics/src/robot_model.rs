use std::fmt::Debug;
use std::marker::PhantomData;
use ad_trait::*;
use nalgebra::Isometry3;
use urdf_rs::{Collision, Color, Dynamics, Geometry, Inertial, Joint, JointLimit, JointType, Link, Material, Mimic, Pose, SafetyController, Texture, Visual};
use serde::{Serialize, Deserialize};
use serde::de::DeserializeOwned;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_utils::arr_storage::*;
use crate::utils::get_urdf_path_from_robot_name;
use serde_with::*;
use optima_3d_spatial::optima_3d_pose::SerdeO3DPose;

// pub type ORobotModelDefault<T> = ORobotModel<T, Isometry3<T>, ArrayVecStor<100>, 100, 100>;

pub type ORobotModelDefault<T> = ORobotModel<T, Isometry3<T>, ORobotArrStorageArrayVec<100, 100, 65>>;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ORobotModel<T: AD, P: O3DPose<T>, S: ORobotArrStorageTrait> {
    #[serde(deserialize_with = "<<S as ORobotArrStorageTrait>::LinksArrType as ArrStorageTrait>::ArrType::deserialize")]
    links: <S::LinksArrType as ArrStorageTrait>::ArrType<OLink<T, P, S>>,
    #[serde(deserialize_with = "<<S as ORobotArrStorageTrait>::JointsArrType as ArrStorageTrait>::ArrType::deserialize")]
    joints: <S::JointsArrType as ArrStorageTrait>::ArrType<OJoint<T, P, S>>,
    phantom_data: PhantomData<(T, P)>
}
impl<T: AD, P: O3DPose<T>, S: ORobotArrStorageTrait> ORobotModel<T, P, S> {
    pub fn from_urdf(robot_name: &str) -> Self {
        let urdf_path = get_urdf_path_from_robot_name(robot_name);
        let urdf = urdf_path.load_urdf();

        let mut links = vec![];
        let mut joints = vec![];

        urdf.links.iter().for_each(|x| {
            links.push(OLink::from_link(x));
        });

        urdf.joints.iter().for_each(|x| {
            joints.push(OJoint::from_joint(x));
        });

        Self {
            links: <S::LinksArrType as ArrStorageTrait>::ArrType::from_slice(&links),
            joints: <S::JointsArrType as ArrStorageTrait>::ArrType::from_slice(&joints),
            phantom_data: Default::default(),
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ORobotArrStorageTrait: Clone + Debug + DeserializeOwned + Serialize {
    type LinksArrType: ArrStorageTrait;
    type JointsArrType: ArrStorageTrait;
    type LinkNameStrType: StrStorageTrait;
    type CollisionArrType: ArrStorageTrait;
    type VisualArrType: ArrStorageTrait;
    type JointNameStrType: StrStorageTrait;
    type ParentLinkStrType: StrStorageTrait;
    type ChildLinkStrType: StrStorageTrait;
    type CollisionNameStrType: StrStorageTrait;
    type VisualNameStrType: StrStorageTrait;
    type MaterialNameStrType: StrStorageTrait;
    type TextureFilenameStrType: StrStorageTrait;
    type GeometryFilenameStrType: StrStorageTrait;
    type MimicJointNameStrType: StrStorageTrait;
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ORobotArrStorageVec;
impl ORobotArrStorageTrait for ORobotArrStorageVec {
    type LinksArrType = VecStor;
    type JointsArrType = VecStor;
    type LinkNameStrType = StringStor;
    type CollisionArrType = VecStor;
    type VisualArrType = VecStor;
    type JointNameStrType = StringStor;
    type ParentLinkStrType = StringStor;
    type ChildLinkStrType = StringStor;
    type CollisionNameStrType = StringStor;
    type VisualNameStrType = StringStor;
    type MaterialNameStrType = StringStor;
    type TextureFilenameStrType = StringStor;
    type GeometryFilenameStrType = StringStor;
    type MimicJointNameStrType = StringStor;
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ORobotArrStorageArrayVec<const NUM_LINKS: usize, const NUM_JOINTS: usize, const NAME_STRING_LEN: usize>;
impl<const NUM_LINKS: usize, const NUM_JOINTS: usize, const NAME_STRING_LEN: usize> ORobotArrStorageTrait for ORobotArrStorageArrayVec<NUM_LINKS, NUM_JOINTS, NAME_STRING_LEN> {
    type LinksArrType = ArrayVecStor<NUM_LINKS>;
    type JointsArrType = ArrayVecStor<NUM_JOINTS>;
    type LinkNameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type CollisionArrType = ArrayVecStor<2>;
    type VisualArrType = ArrayVecStor<2>;
    type JointNameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type ParentLinkStrType = ArrayStringStor<NAME_STRING_LEN>;
    type ChildLinkStrType = ArrayStringStor<NAME_STRING_LEN>;
    type CollisionNameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type VisualNameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type MaterialNameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type TextureFilenameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type GeometryFilenameStrType = ArrayStringStor<NAME_STRING_LEN>;
    type MimicJointNameStrType = ArrayStringStor<NAME_STRING_LEN>;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OLink<T: AD, P: O3DPose<T>, S: ORobotArrStorageTrait> {
    link_idx: usize,
    #[serde(deserialize_with = "<<S as ORobotArrStorageTrait>::LinkNameStrType as StrStorageTrait>::StrType::deserialize")]
    name: <S::LinkNameStrType as StrStorageTrait>::StrType,
    #[serde(deserialize_with = "<<S as ORobotArrStorageTrait>::CollisionArrType as ArrStorageTrait>::ArrType::deserialize")]
    collision: <S::CollisionArrType as ArrStorageTrait>::ArrType<OCollision<T, P, S>>,
    #[serde(deserialize_with = "<<S as ORobotArrStorageTrait>::VisualArrType as ArrStorageTrait>::ArrType::deserialize")]
    visual: <S::VisualArrType as ArrStorageTrait>::ArrType<OVisual<T, P, S>>,
    #[serde(deserialize_with = "OInertial::<T>::deserialize")]
    inertial: OInertial<T>
}
impl<T: AD, P: O3DPose<T>, S: ORobotArrStorageTrait> OLink<T, P, S> {
    fn from_link(link: &Link) -> Self {
        Self {
            link_idx: 0,
            name: <S::LinkNameStrType as StrStorageTrait>::StrType::from_str(&link.name),
            collision: link.collision.iter().map(|x| OCollision::from_collision(x)).collect(),
            visual: link.visual.iter().map(|x| OVisual::from_visual(x)).collect(),
            inertial: OInertial::from_inertial(&link.inertial)
        }
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OJoint<T: AD, P: O3DPose<T>, S: ORobotArrStorageTrait> {
    joint_idx: usize,
    #[serde(deserialize_with = "<<S as ORobotArrStorageTrait>::JointNameStrType as StrStorageTrait>::StrType::deserialize")]
    name: <S::JointNameStrType as StrStorageTrait>::StrType,
    joint_type: OJointType,
    #[serde(deserialize_with = "OPose::<T, P>::deserialize")]
    origin: OPose<T, P>,
    #[serde_as(as = "[SerdeAD<T>; 3]")]
    axis: [T; 3],
    #[serde(deserialize_with = "<<S as ORobotArrStorageTrait>::ParentLinkStrType as StrStorageTrait>::StrType::deserialize")]
    parent_link: <S::ParentLinkStrType as StrStorageTrait>::StrType,
    #[serde(deserialize_with = "<<S as ORobotArrStorageTrait>::ChildLinkStrType as StrStorageTrait>::StrType::deserialize")]
    child_link: <S::ChildLinkStrType as StrStorageTrait>::StrType,
    #[serde(deserialize_with = "OJointLimit::<T>::deserialize")]
    limit: OJointLimit<T>,
    #[serde(deserialize_with = "Option::<ODynamics::<T>>::deserialize")]
    dynamics: Option<ODynamics<T>>,
    #[serde(deserialize_with = "Option::<OMimic<T, _>>::deserialize")]
    mimic: Option<OMimic<T, S>>,
    #[serde(deserialize_with = "Option::<OSafetyController<T>>::deserialize")]
    safety_controller: Option<OSafetyController<T>>
}
impl<T: AD, P: O3DPose<T>, S: ORobotArrStorageTrait> OJoint<T, P, S> {
    fn from_joint(joint: &Joint) -> Self {
        let axis: Vec<T> = joint.axis.xyz.0.iter().map(|x| T::constant(*x)).collect();

        Self {
            joint_idx: 0,
            name: <S::JointNameStrType as StrStorageTrait>::StrType::from_str(&joint.name),
            joint_type: OJointType::from_joint_type(&joint.joint_type),
            origin: OPose::from_pose(&joint.origin),
            axis: [axis[0], axis[1], axis[2]],
            parent_link: <S::ParentLinkStrType as StrStorageTrait>::StrType::from_str(&joint.parent.link),
            child_link: <S::ChildLinkStrType as StrStorageTrait>::StrType::from_str(&joint.child.link),
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
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OInertial<T: AD> {
    #[serde_as(as = "SerdeAD<T>")]
    ixx: T,
    #[serde_as(as = "SerdeAD<T>")]
    ixy: T,
    #[serde_as(as = "SerdeAD<T>")]
    ixz: T,
    #[serde_as(as = "SerdeAD<T>")]
    iyy: T,
    #[serde_as(as = "SerdeAD<T>")]
    iyz: T,
    #[serde_as(as = "SerdeAD<T>")]
    izz: T
}
impl<T: AD> OInertial<T> {
    fn from_inertial(inertial: &Inertial) -> Self {
        Self {
            ixx: T::constant(inertial.inertia.ixx),
            ixy: T::constant(inertial.inertia.ixy),
            ixz: T::constant(inertial.inertia.ixz),
            iyy: T::constant(inertial.inertia.iyy),
            iyz: T::constant(inertial.inertia.iyz),
            izz: T::constant(inertial.inertia.izz)
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OCollision<T: AD, P: O3DPose<T>, S: ORobotArrStorageTrait> {
    #[serde(deserialize_with = "OGeometry::<S>::deserialize")]
    geometry: OGeometry<S>,
    #[serde(deserialize_with = "Option::<<<S as ORobotArrStorageTrait>::CollisionNameStrType as StrStorageTrait>::StrType>::deserialize")]
    name: Option<<S::CollisionNameStrType as StrStorageTrait>::StrType>,
    #[serde(deserialize_with = "OPose::<T, P>::deserialize")]
    origin: OPose<T, P>
}
impl<T: AD, P: O3DPose<T>, S: ORobotArrStorageTrait> OCollision<T, P, S> {
    fn from_collision(collision: &Collision) -> Self {
        Self {
            geometry: OGeometry::from_geometry(&collision.geometry),
            name: match &collision.name {
                None => { None }
                Some(name) => { Some(<S::CollisionNameStrType as StrStorageTrait>::StrType::from_str(name)) }
            },
            origin: OPose::from_pose(&collision.origin)
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OVisual<T: AD, P: O3DPose<T>, S: ORobotArrStorageTrait> {
    #[serde(deserialize_with = "Option::<<<S as ORobotArrStorageTrait>::VisualNameStrType as StrStorageTrait>::StrType>::deserialize")]
    name: Option<<S::VisualNameStrType as StrStorageTrait>::StrType>,
    #[serde(deserialize_with = "Option::<OMaterial<S>>::deserialize")]
    material: Option<OMaterial<S>>,
    #[serde(deserialize_with = "OPose::<T, P>::deserialize")]
    origin: OPose<T, P>,
    #[serde(deserialize_with = "OGeometry::<S>::deserialize")]
    geometry: OGeometry<S>
}
impl<T: AD, P: O3DPose<T>, S: ORobotArrStorageTrait> OVisual<T, P, S> {
    fn from_visual(visual: &Visual) -> Self {
        Self {
            name: match &visual.name {
                None => { None }
                Some(name) => { Some(<S::VisualNameStrType as StrStorageTrait>::StrType::from_str(name)) }
            },
            material: match &visual.material {
                None => { None }
                Some(material) => { Some(OMaterial::from_material(material)) }
            },
            origin: OPose::from_pose(&visual.origin),
            geometry: OGeometry::from_geometry(&visual.geometry)
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OMaterial<S: ORobotArrStorageTrait> {
    #[serde(deserialize_with = "Option::<OTexture<S>>::deserialize")]
    texture: Option<OTexture<S>>,
    #[serde(deserialize_with = "<<S as ORobotArrStorageTrait>::MaterialNameStrType as StrStorageTrait>::StrType::deserialize")]
    name: <S::MaterialNameStrType as StrStorageTrait>::StrType,
    color: Option<OColor>,
    _phantom_data: PhantomData<S>
}
impl<S: ORobotArrStorageTrait> OMaterial<S> {
    fn from_material(material: &Material) -> Self {
        Self {
            texture: match &material.texture {
                None => { None }
                Some(texture) => { Some(OTexture::from_texture(texture)) }
            },
            name: <S::MaterialNameStrType as StrStorageTrait>::StrType::from_str(&material.name),
            color: match &material.color {
                None => { None }
                Some(color) => { Some(OColor::from_color(color)) }
            },
            _phantom_data: Default::default()
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OTexture<S: ORobotArrStorageTrait> {
    filename: <S::TextureFilenameStrType as StrStorageTrait>::StrType,
    _phantom_data: PhantomData<S>
}
impl<S: ORobotArrStorageTrait> OTexture<S> {
    fn from_texture(texture: &Texture) -> Self {
        Self {
            filename: <S::TextureFilenameStrType as StrStorageTrait>::StrType::from_str(&texture.filename),
            _phantom_data: Default::default()
        }
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
pub enum OGeometry<S: ORobotArrStorageTrait> {
    Box { size: [f64; 3] },
    Cylinder { radius: f64, length: f64 },
    Capsule { radius: f64, length: f64 },
    Sphere { radius: f64 },
    Mesh { filename: <S::GeometryFilenameStrType as StrStorageTrait>::StrType, scale: Option<[f64; 3]> }
}
impl<S: ORobotArrStorageTrait> OGeometry<S> {
    fn from_geometry(geometry: &Geometry) -> Self {
        return match &geometry {
            Geometry::Box { size } => { OGeometry::Box { size: size.0 } }
            Geometry::Cylinder { radius, length } => { OGeometry::Cylinder { radius: *radius, length: *length } }
            Geometry::Capsule { radius, length } => { OGeometry::Capsule { radius: *radius, length: *length } }
            Geometry::Sphere { radius } => { OGeometry::Sphere { radius: *radius } }
            Geometry::Mesh { filename, scale } => { OGeometry::Mesh { filename: <S::GeometryFilenameStrType as StrStorageTrait>::StrType::from_str(filename), scale: match scale { None => { None } Some(scale) => { Some(scale.0) } } } }
        }
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OPose<T: AD, P: O3DPose<T>> {
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    pose: P,
    #[serde_as(as = "[SerdeAD<T>; 3]")]
    rpy: [T; 3],
    #[serde_as(as = "[SerdeAD<T>; 3]")]
    xyz: [T; 3]
}
impl<T: AD, P: O3DPose<T>> OPose<T, P> {
    fn from_pose(pose: &Pose) -> Self {
        let rpy: Vec<T> = pose.rpy.0.iter().map(|x| T::constant(*x)).collect();
        let xyz: Vec<T> = pose.xyz.0.iter().map(|x| T::constant(*x)).collect();

        let pose = P::from_translation_and_rotation_constructor(&xyz, &rpy);

        Self {
            pose,
            rpy: [rpy[0], rpy[1], rpy[2]],
            xyz: [xyz[0], xyz[1], xyz[2]]
        }
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

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OMimic<T: AD, S: ORobotArrStorageTrait> {
    joint: <S::MimicJointNameStrType as StrStorageTrait>::StrType,
    #[serde_as(as = "Option<SerdeAD<T>>")]
    multiplier: Option<T>,
    #[serde_as(as = "Option<SerdeAD<T>>")]
    offset: Option<T>
}
impl<T: AD, S: ORobotArrStorageTrait> OMimic<T, S> {
    fn from_mimic(mimic: &Mimic) -> Self {
        Self {
            joint: <S::MimicJointNameStrType as StrStorageTrait>::StrType::from_str(&mimic.joint),
            multiplier: match &mimic.multiplier {
                None => { None }
                Some(multiplier) => { Some(T::constant(*multiplier)) }
            },
            offset: match &mimic.offset {
                None => { None }
                Some(offset) => { Some(T::constant(*offset)) }
            }
        }
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ODynamics<T: AD> {
    #[serde_as(as = "SerdeAD<T>")]
    damping: T,
    #[serde_as(as = "SerdeAD<T>")]
    friction: T
}
impl<T: AD> ODynamics<T> {
    fn from_dynamics(dynamics: &Dynamics) -> Self {
        Self {
            damping: T::constant(dynamics.damping),
            friction: T::constant(dynamics.friction)
        }
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OJointLimit<T: AD> {
    #[serde_as(as = "SerdeAD<T>")]
    effort: T,
    #[serde_as(as = "SerdeAD<T>")]
    lower: T,
    #[serde_as(as = "SerdeAD<T>")]
    upper: T,
    #[serde_as(as = "SerdeAD<T>")]
    velocity: T
}
impl<T: AD> OJointLimit<T> {
    fn from_joint_limit(joint_limit: &JointLimit) -> Self {
        Self {
            effort:   T::constant(joint_limit.effort),
            lower:    T::constant(joint_limit.lower),
            upper:    T::constant(joint_limit.upper),
            velocity: T::constant(joint_limit.velocity)
        }
    }
}

#[serde_as]
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OSafetyController<T: AD> {
    #[serde_as(as = "SerdeAD<T>")]
    k_position: T,
    #[serde_as(as = "SerdeAD<T>")]
    k_velocity: T,
    #[serde_as(as = "SerdeAD<T>")]
    soft_lower_limit: T,
    #[serde_as(as = "SerdeAD<T>")]
    soft_upper_limit: T
}
impl<T: AD> OSafetyController<T> {
    fn from_safety_controller(safety_controller: &SafetyController) -> Self {
        Self {
            k_position:       T::constant(safety_controller.k_position),
            k_velocity:       T::constant(safety_controller.k_velocity),
            soft_lower_limit: T::constant(safety_controller.soft_lower_limit),
            soft_upper_limit: T::constant(safety_controller.soft_upper_limit)
        }
    }
}