use std::borrow::Cow;
use std::cmp::Ordering;
use ad_trait::AD;
use parry_ad::na::Isometry3;
use parry_ad::query::Contact;
use parry_ad::shape::Shape;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use crate::parry::parry_shapes::{OParryShp, OParryShpCowTrait, OParryShpGenericTrait, OParryShpHierarchyTrait};
use crate::shape_query_traits::{OPairContactQryTrait, OPairDisQryTrait, OPairIntersectQryTrait, OPairShpQryTrait, OSingleShpQryTrait};


pub struct OParryRawIntersectQry;
impl<T: AD> OPairShpQryTrait<T> for OParryRawIntersectQry {
    type ShapeType = Box<dyn Shape<T>>;
    type PoseType = Isometry3<T>;
    type Args = ();
    type Output = bool;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, _args: &Self::Args) -> Self::Output {
        parry_ad::query::intersection_test(pose_a, shape_a.as_ref(), pose_b, shape_b.as_ref()).expect("error")
    }
}
impl OPairIntersectQryTrait for OParryRawIntersectQry { }

pub struct OParryRawDisQry;
impl<T: AD> OPairShpQryTrait<T> for OParryRawDisQry {
    type ShapeType = Box<dyn Shape<T>>;
    type PoseType = Isometry3<T>;
    type Args = ();
    type Output = T;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, _args: &Self::Args) -> Self::Output {
        parry_ad::query::distance(pose_a, shape_a.as_ref(), pose_b, shape_b.as_ref()).expect("error")
    }
}
impl OPairDisQryTrait for OParryRawDisQry { }

pub struct OParryRawContactQry;
impl<T: AD> OPairShpQryTrait<T> for OParryRawContactQry {
    type ShapeType = Box<dyn Shape<T>>;
    type PoseType = Isometry3<T>;
    type Args = T;
    type Output = ParryContactWrapper<T>;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, args: &Self::Args) -> Self::Output {
        ParryContactWrapper(parry_ad::query::contact(pose_a, shape_a.as_ref(), pose_b, shape_b.as_ref(), *args).expect("error"))
    }
}
impl OPairContactQryTrait for OParryRawContactQry { }

pub struct OParryRawDisViaContactQry;
impl<T: AD> OPairShpQryTrait<T> for OParryRawDisViaContactQry {
    type ShapeType = Box<dyn Shape<T>>;
    type PoseType = Isometry3<T>;
    type Args = ();
    type Output = T;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, _args: &Self::Args) -> Self::Output {
        let binding = OParryRawContactQry::query(shape_a, shape_b, pose_a, pose_b, &T::constant(f64::MAX));
        let contact = binding.0.as_ref().unwrap();
        contact.dist
    }
}
impl OPairDisQryTrait for OParryRawDisViaContactQry { }

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct OParryIntersectQry;
pub struct OParryIntersectQryArgs { pub rep_a: ParryShapeRepresentation, pub rep_b: ParryShapeRepresentation }
impl<T: AD> OPairShpQryTrait<T> for OParryIntersectQry {
    type ShapeType = OParryShp<T, Isometry3<T>>;
    type PoseType = Isometry3<T>;
    type Args = OParryIntersectQryArgs;
    type Output = bool;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, args: &Self::Args) -> Self::Output {
        parry_pairwise_shape_raw_query::<_, _, _, OParryRawIntersectQry>(shape_a, pose_a, &args.rep_a, shape_b, pose_b, &args.rep_b, &())
    }
}
impl OPairIntersectQryTrait for OParryIntersectQry { }

pub struct OParryDisQry;
pub struct OParryDisQryArgs { pub rep_a: ParryShapeRepresentation, pub rep_b: ParryShapeRepresentation }
impl<T: AD> OPairShpQryTrait<T> for OParryDisQry {
    type ShapeType = OParryShp<T, Isometry3<T>>;
    type PoseType = Isometry3<T>;
    type Args = OParryDisQryArgs;
    type Output = T;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, args: &Self::Args) -> Self::Output {
        parry_pairwise_shape_raw_query::<_, _, _, OParryRawDisQry>(shape_a, pose_a, &args.rep_a, shape_b, pose_b, &args.rep_b, &())
    }
}
impl OPairDisQryTrait for OParryDisQry { }

pub struct OParryContactQry;
pub struct OParryContactQryArgs<T: AD> { pub rep_a: ParryShapeRepresentation, pub rep_b: ParryShapeRepresentation, pub threshold: T }
impl<T: AD> OPairShpQryTrait<T> for OParryContactQry {
    type ShapeType = OParryShp<T, Isometry3<T>>;
    type PoseType = Isometry3<T>;
    type Args = OParryContactQryArgs<T>;
    type Output = ParryContactWrapper<T>;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, args: &Self::Args) -> Self::Output {
        parry_pairwise_shape_raw_query::<_, _, _, OParryRawContactQry>(shape_a, pose_a, &args.rep_a, shape_b, pose_b, &args.rep_b, &args.threshold)
    }
}
impl OPairContactQryTrait for OParryContactQry { }

////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug)] pub enum ParryShapeRepresentation {
    Full,
    BoundingSphere,
    OBB
}

pub (crate) fn parry_pairwise_shape_raw_query<T, P, S, Q>(shape_a: &S, pose_a: &P, rep_a: &ParryShapeRepresentation, shape_b: &S, pose_b: &P, rep_b: &ParryShapeRepresentation, args: &Q::Args) -> Q::Output
    where T: AD,
          P: O3DPose<T>,
          S: OParryShpHierarchyTrait<T, P>,
          Q: OPairShpQryTrait<T, ShapeType = Box<dyn Shape<T>>, PoseType=Isometry3<T>>
{
    let (shape_a, pose_a) = parry_get_shape_and_pose(shape_a, pose_a, rep_a);
    let (shape_b, pose_b) = parry_get_shape_and_pose(shape_b, pose_b, rep_b);

    Q::query(shape_a, shape_b, pose_a.as_ref(), pose_b.as_ref(), args)
}

#[allow(dead_code)]
pub (crate) fn parry_single_shape_raw_query<T, P, S, Q>(shape: &S, pose: &P, rep: &ParryShapeRepresentation, args: &Q::Args) -> Q::Output
    where T: AD,
          P: O3DPose<T>,
          S: OParryShpHierarchyTrait<T, P>,
          Q: OSingleShpQryTrait<T, ShapeType = Box<dyn Shape<T>>, PoseType=Isometry3<T>>
{
    let (shape_a, pose_a) = parry_get_shape_and_pose(shape, pose, rep);

    Q::query(shape_a, pose_a.as_ref(), args)
}

pub (crate) fn parry_get_shape_and_pose<'a, T: AD, P: O3DPose<T>, S: OParryShpHierarchyTrait<T, P>>(shape: &'a S, pose: &'a P, rep: &'a ParryShapeRepresentation) -> (&'a Box<dyn Shape<T>>, Cow<'a, Isometry3<T>>) {
    return match rep {
        ParryShapeRepresentation::Full => {
            let s = shape.shape();
            let p = shape.get_isometry3_cow(pose);
            (s, p)
        }
        ParryShapeRepresentation::BoundingSphere => {
            let s = shape.bounding_sphere();
            let p = s.get_isometry3_cow(pose);
            (s.shape(), p)
        }
        ParryShapeRepresentation::OBB => {
            let s = shape.obb();
            let p = s.get_isometry3_cow(pose);
            (s.shape(), p)
        }
    }
}

#[derive(Clone, Debug)]
pub struct ParryContactWrapper<T: AD>(pub Option<Contact<T>>);
impl<T: AD> PartialEq for ParryContactWrapper<T> {
    fn eq(&self, other: &Self) -> bool {
        match (&self.0, &other.0) {
            (Some(a), Some(b)) => { a.dist.eq(&b.dist) }
            (Some(_), None) => { false }
            (None, Some(_)) => { false }
            (None, None) => { true }
        }
    }
}
impl<T: AD> PartialOrd for ParryContactWrapper<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match (&self.0, &other.0) {
            (Some(a), Some(b)) => { a.dist.partial_cmp(&b.dist) }
            (Some(_), None) => { Some(Ordering::Greater) }
            (None, Some(_)) => { Some(Ordering::Less) }
            (None, None) => { Some(Ordering::Equal) }
        }
    }
}