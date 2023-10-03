use std::borrow::Cow;
use std::cmp::Ordering;
use ad_trait::AD;
use parry_ad::na::Isometry3;
use parry_ad::query::Contact;
use parry_ad::shape::Shape;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use crate::parry::parry_shapes::{OShapeParry, OShapeParryCowTrait, OShapeParryGenericTrait, OShapeParryHierarchyTrait};
use crate::shape_query_traits::{OPairwiseContactQueryTrait, OPairwiseDistanceQueryTrait, OPairwiseIntersectionQueryTrait, OPairwiseShapeQueryTrait, OParryPairwiseShapeQueryTrait, OSingleShapeQueryTrait};


pub struct OParryRawIntersectQuery;
impl<T: AD> OPairwiseShapeQueryTrait<T> for OParryRawIntersectQuery {
    type ShapeType = Box<dyn Shape<T>>;
    type PoseType = Isometry3<T>;
    type Args = ();
    type Output = bool;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, _args: &Self::Args) -> Self::Output {
        parry_ad::query::intersection_test(pose_a, shape_a.as_ref(), pose_b, shape_b.as_ref()).expect("error")
    }
}
impl OParryPairwiseShapeQueryTrait for OParryRawIntersectQuery { }
impl OPairwiseIntersectionQueryTrait for OParryRawIntersectQuery { }

pub struct OParryRawDistanceQuery;
impl<T: AD> OPairwiseShapeQueryTrait<T> for OParryRawDistanceQuery {
    type ShapeType = Box<dyn Shape<T>>;
    type PoseType = Isometry3<T>;
    type Args = ();
    type Output = T;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, _args: &Self::Args) -> Self::Output {
        parry_ad::query::distance(pose_a, shape_a.as_ref(), pose_b, shape_b.as_ref()).expect("error")
    }
}
impl OParryPairwiseShapeQueryTrait for OParryRawDistanceQuery { }
impl OPairwiseDistanceQueryTrait for OParryRawDistanceQuery { }

pub struct OParryRawContactQuery;
impl<T: AD> OPairwiseShapeQueryTrait<T> for OParryRawContactQuery {
    type ShapeType = Box<dyn Shape<T>>;
    type PoseType = Isometry3<T>;
    type Args = T;
    type Output = ParryContactWrapper<T>;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, args: &Self::Args) -> Self::Output {
        ParryContactWrapper(parry_ad::query::contact(pose_a, shape_a.as_ref(), pose_b, shape_b.as_ref(), *args).expect("error"))
    }
}
impl OParryPairwiseShapeQueryTrait for OParryRawContactQuery { }
impl OPairwiseContactQueryTrait for OParryRawContactQuery { }

pub struct OParryRawDistanceViaContactQuery;
impl<T: AD> OPairwiseShapeQueryTrait<T> for OParryRawDistanceViaContactQuery {
    type ShapeType = Box<dyn Shape<T>>;
    type PoseType = Isometry3<T>;
    type Args = ();
    type Output = T;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, _args: &Self::Args) -> Self::Output {
        let binding = OParryRawContactQuery::query(shape_a, shape_b, pose_a, pose_b, &T::constant(f64::MAX));
        let contact = binding.0.as_ref().unwrap();
        contact.dist
    }
}
impl OParryPairwiseShapeQueryTrait for OParryRawDistanceViaContactQuery { }
impl OPairwiseDistanceQueryTrait for OParryRawDistanceViaContactQuery { }

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct OParryIntersectQuery;
pub struct OParryIntersectQueryArgs { pub rep_a: ParryShapeRepresentation, pub rep_b: ParryShapeRepresentation }
impl<T: AD> OPairwiseShapeQueryTrait<T> for OParryIntersectQuery {
    type ShapeType = OShapeParry<T, Isometry3<T>>;
    type PoseType = Isometry3<T>;
    type Args = OParryIntersectQueryArgs;
    type Output = bool;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, args: &Self::Args) -> Self::Output {
        parry_pairwise_shape_query::<_, _, _, OParryRawIntersectQuery>(shape_a, pose_a, &args.rep_a, shape_b, pose_b, &args.rep_b, &())
    }
}
impl OParryPairwiseShapeQueryTrait for OParryIntersectQuery { }
impl OPairwiseIntersectionQueryTrait for OParryIntersectQuery { }

pub struct OParryDistanceQuery;
pub struct OParryDistanceQueryArgs { pub rep_a: ParryShapeRepresentation, pub rep_b: ParryShapeRepresentation }
impl<T: AD> OPairwiseShapeQueryTrait<T> for OParryDistanceQuery {
    type ShapeType = OShapeParry<T, Isometry3<T>>;
    type PoseType = Isometry3<T>;
    type Args = OParryDistanceQueryArgs;
    type Output = T;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, args: &Self::Args) -> Self::Output {
        parry_pairwise_shape_query::<_, _, _, OParryRawDistanceQuery>(shape_a, pose_a, &args.rep_a, shape_b, pose_b, &args.rep_b, &())
    }
}
impl OParryPairwiseShapeQueryTrait for OParryDistanceQuery { }
impl OPairwiseDistanceQueryTrait for OParryDistanceQuery { }

pub struct OParryContactQuery;
pub struct OParryContactQueryArgs<T: AD> { pub rep_a: ParryShapeRepresentation, pub rep_b: ParryShapeRepresentation, pub threshold: T }
impl<T: AD> OPairwiseShapeQueryTrait<T> for OParryContactQuery {
    type ShapeType = OShapeParry<T, Isometry3<T>>;
    type PoseType = Isometry3<T>;
    type Args = OParryContactQueryArgs<T>;
    type Output = ParryContactWrapper<T>;

    fn query(shape_a: &Self::ShapeType, shape_b: &Self::ShapeType, pose_a: &Self::PoseType, pose_b: &Self::PoseType, args: &Self::Args) -> Self::Output {
        parry_pairwise_shape_query::<_, _, _, OParryRawContactQuery>(shape_a, pose_a, &args.rep_a, shape_b, pose_b, &args.rep_b, &args.threshold)
    }
}
impl OParryPairwiseShapeQueryTrait for OParryContactQuery { }
impl OPairwiseContactQueryTrait for OParryContactQuery { }

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug)] pub enum ParryShapeRepresentation {
    Full,
    BoundingSphere,
    OBB
}

pub (crate) fn parry_pairwise_shape_query<T, P, S, Q>(shape_a: &S, pose_a: &P, rep_a: &ParryShapeRepresentation, shape_b: &S, pose_b: &P, rep_b: &ParryShapeRepresentation, args: &Q::Args) -> Q::Output
    where T: AD,
          P: O3DPose<T>,
          S: OShapeParryHierarchyTrait<T, P>,
          Q: OPairwiseShapeQueryTrait<T, ShapeType = Box<dyn Shape<T>>, PoseType=Isometry3<T>> + OParryPairwiseShapeQueryTrait
{
    let (shape_a, pose_a) = parry_get_shape_and_pose(shape_a, pose_a, rep_a);
    let (shape_b, pose_b) = parry_get_shape_and_pose(shape_b, pose_b, rep_b);

    Q::query(shape_a, shape_b, pose_a.as_ref(), pose_b.as_ref(), args)
}

#[allow(dead_code)]
pub (crate) fn parry_single_shape_query<T, P, S, Q>(shape: &S, pose: &P, rep: &ParryShapeRepresentation, args: &Q::Args) -> Q::Output
    where T: AD,
          P: O3DPose<T>,
          S: OShapeParryHierarchyTrait<T, P>,
          Q: OSingleShapeQueryTrait<T, ShapeType = Box<dyn Shape<T>>, PoseType=Isometry3<T>> + OParryPairwiseShapeQueryTrait
{
    let (shape_a, pose_a) = parry_get_shape_and_pose(shape, pose, rep);

    Q::query(shape_a, pose_a.as_ref(), args)
}

pub (crate) fn parry_get_shape_and_pose<'a, T: AD, P: O3DPose<T>, S: OShapeParryHierarchyTrait<T, P>>(shape: &'a S, pose: &'a P, rep: &'a ParryShapeRepresentation) -> (&'a Box<dyn Shape<T>>, Cow<'a, Isometry3<T>>) {
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