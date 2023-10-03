pub mod shape_query_traits;
pub mod shape_group_query_traits;
pub mod parry;


/*
use std::borrow::Cow;
use std::cmp::Ordering;
use ad_trait::AD;
use as_any::{AsAny};
use parry_ad::na::{Isometry3, Point3, Vector3};
use parry_ad::query;
use parry_ad::query::Contact;
use parry_ad::shape::{Ball, Cuboid, Shape};
use optima_3d_mesh::OTriMesh;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_3d_spatial::optima_3d_vec::{O3DVec, O3DVecCategoryPoint3};

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait OShapeParryGenericTrait<T: AD> {
    fn shape(&self) -> &Box<dyn Shape<T>>;
    fn get_isometry3_cow<'a, P: O3DPose<T>>(&self, pose: &'a P) -> Cow<'a, Isometry3<T>>;
}

pub trait OShapeParryHierarchyTrait<T: AD, P: O3DPose<T>> : OShapeParryGenericTrait<T> {
    fn bounding_sphere(&self) -> &OShapeParryGenericWithOffset<T, P>;
    fn obb(&self) -> &OShapeParryGenericWithOffset<T, P>;
}

pub struct OShapeParryGeneric<T: AD> {
    shape: Box<dyn Shape<T>>
}
impl<T: AD> OShapeParryGeneric<T> {
    pub fn new<S: Shape<T>>(shape: S) -> Self {
        Self {
            shape: Box::new(shape)
        }
    }
}
impl<T: AD> OShapeParryGenericTrait<T> for OShapeParryGeneric<T> {
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }

    #[inline]
    fn get_isometry3_cow<'a, P: O3DPose<T>>(&self, pose: &'a P) -> Cow<'a, Isometry3<T>> {
        pose.downcast_or_convert::<Isometry3<T>>()
    }
}

pub struct OShapeParryGenericWithOffset<T: AD, P: O3DPose<T>> {
    shape: Box<dyn Shape<T>>,
    offset: P
}
impl<T: AD, P: O3DPose<T>> OShapeParryGenericWithOffset<T, P> {
    pub fn new<S: Shape<T>>(shape: S, offset: P) -> Self {
        Self {
            shape: Box::new(shape),
            offset,
        }
    }
    #[inline(always)]
    pub fn offset(&self) -> &P {
        &self.offset
    }
}
impl<T: AD, P: O3DPose<T>> OShapeParryGenericTrait<T> for OShapeParryGenericWithOffset<T, P> {
    #[inline]
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }

    #[inline]
    fn get_isometry3_cow<'a, P2: O3DPose<T>>(&self, pose: &'a P2) -> Cow<'a, Isometry3<T>> {
        let offset: Cow<P2> = self.offset.downcast_or_convert::<P2>();
        let pose = pose.mul(offset.as_ref());
        let res = pose.downcast_or_convert::<Isometry3<T>>();
        Cow::Owned(res.into_owned())
    }
}

pub struct OShapeParryHierarchy<T: AD, P: O3DPose<T>> {
    shape: Box<dyn Shape<T>>,
    bounding_sphere: OShapeParryGenericWithOffset<T, P>,
    obb: OShapeParryGenericWithOffset<T, P>
}
impl<T: AD, P: O3DPose<T>> OShapeParryHierarchy<T, P> {
    pub fn new<S: Shape<T>>(shape: S) -> Self {
        let shape = Box::new(shape);
        let bounding_sphere = get_bounding_sphere_from_shape(&shape, &P::identity());
        let obb = get_obb_from_shape(&shape, &P::identity());

        Self {
            shape,
            bounding_sphere,
            obb
        }
    }
    pub fn new_from_convex_shape(trimesh: &OTriMesh) -> Self {
        let points: Vec<Point3<T>> = trimesh.points().iter().map(|x| x.to_other_generic_category::<T, O3DVecCategoryPoint3>() ).collect();
        let convex_polyhedron = parry_ad::shape::ConvexPolyhedron::from_convex_hull(&points).expect("not a convex shape");
        Self::new(convex_polyhedron)
    }
}
impl<T: AD, P: O3DPose<T>> OShapeParryGenericTrait<T> for OShapeParryHierarchy<T, P> {
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }

    #[inline(always)]
    fn get_isometry3_cow<'a, P2: O3DPose<T>>(&self, pose: &'a P2) -> Cow<'a, Isometry3<T>> {
        pose.downcast_or_convert::<Isometry3<T>>()
    }
}
impl<T: AD, P: O3DPose<T>> OShapeParryHierarchyTrait<T, P> for OShapeParryHierarchy<T, P> {
    #[inline(always)]
    fn bounding_sphere(&self) -> &OShapeParryGenericWithOffset<T, P> {
        &self.bounding_sphere
    }

    #[inline(always)]
    fn obb(&self) -> &OShapeParryGenericWithOffset<T, P> {
        &self.obb
    }
}

pub struct OShapeParryHierarchyWithOffset<T: AD, P: O3DPose<T>> {
    shape: Box<dyn Shape<T>>,
    offset: P,
    bounding_sphere: OShapeParryGenericWithOffset<T, P>,
    obb: OShapeParryGenericWithOffset<T, P>
}
impl<T: AD, P: O3DPose<T>> OShapeParryHierarchyWithOffset<T, P> {
    pub fn new<S: Shape<T>>(shape: S, offset: P) -> Self {
        let shape = Box::new(shape);
        let bounding_sphere = get_bounding_sphere_from_shape(&shape, &offset);
        let obb = get_obb_from_shape(&shape, &offset);

        Self {
            shape,
            offset,
            bounding_sphere,
            obb
        }
    }
    pub fn new_from_convex_shape(trimesh: &OTriMesh, offset: P) -> Self {
        let points: Vec<Point3<T>> = trimesh.points().iter().map(|x| x.to_other_generic_category::<T, O3DVecCategoryPoint3>() ).collect();
        let convex_polyhedron = parry_ad::shape::ConvexPolyhedron::from_convex_hull(&points).expect("not a convex shape");
        Self::new(convex_polyhedron, offset)
    }
}
impl<T: AD, P: O3DPose<T>> OShapeParryGenericTrait<T> for OShapeParryHierarchyWithOffset<T, P> {
    #[inline(always)]
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }

    #[inline(always)]
    fn get_isometry3_cow<'a, P2: O3DPose<T>>(&self, pose: &'a P2) -> Cow<'a, Isometry3<T>> {
        let offset: Cow<P2> = self.offset.downcast_or_convert::<P2>();
        let pose = pose.mul(offset.as_ref());
        let res = pose.downcast_or_convert::<Isometry3<T>>();
        Cow::Owned(res.into_owned())
    }
}
impl<T: AD, P: O3DPose<T>> OShapeParryHierarchyTrait<T, P> for OShapeParryHierarchyWithOffset<T, P> {
    #[inline(always)]
    fn bounding_sphere(&self) -> &OShapeParryGenericWithOffset<T, P> {
        &self.bounding_sphere
    }

    #[inline(always)]
    fn obb(&self) -> &OShapeParryGenericWithOffset<T, P> {
        &self.obb
    }
}

pub (crate) fn get_bounding_sphere_from_shape<T: AD, S: Shape<T>, P: O3DPose<T>>(shape: &Box<S>, offset: &P) -> OShapeParryGenericWithOffset<T, P> {
    let bounding_sphere = shape.compute_local_bounding_sphere();
    let offset = offset.mul(&P::from_constructors(&bounding_sphere.center, &[T::zero();3]));
    let sphere = Ball::new(bounding_sphere.radius);
    OShapeParryGenericWithOffset::new(sphere, offset)
}

pub (crate) fn get_obb_from_shape<T: AD, S: Shape<T>, P: O3DPose<T>>(shape: &Box<S>, offset: &P) -> OShapeParryGenericWithOffset<T, P> {
    let aabb = shape.compute_local_aabb();
    let mins = aabb.mins;
    let maxs = aabb.maxs;
    let center = mins.add(&maxs).scalar_mul_o3dvec(T::constant(0.5));
    let offset = offset.mul(&P::from_constructors(&center, &[T::zero(); 3]));
    let half_x = (maxs[0] - mins[0]) * T::constant(0.5);
    let half_y = (maxs[1] - mins[1]) * T::constant(0.5);
    let half_z = (maxs[2] - mins[2]) * T::constant(0.5);
    let cuboid = Cuboid::new(Vector3::new(half_x, half_y, half_z));
    OShapeParryGenericWithOffset::new(cuboid, offset)
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait OShapeIntersectTrait<T: AD, P: O3DPose<T>, B> {
    fn intersect(&self, self_pose: &P, other: &B, other_pose: &P) -> bool;
}
pub trait OBoundingSphereToBoundingSphereIntersectTrait<T: AD, P: O3DPose<T>, B> {
    fn bounding_sphere_to_bounding_sphere_intersect(&self, self_pose: &P, other: &B, other_pose: &P) -> bool;
}
pub trait OOBBToOBBIntersectTrait<T: AD, P: O3DPose<T>, B> {
    fn obb_to_obb_intersect(&self, self_pose: &P, other: &B, other_pose: &P) -> bool;
}

impl<T: AD, P: O3DPose<T>, A: OShapeParryGenericTrait<T>, B: OShapeParryGenericTrait<T>> OShapeIntersectTrait<T, P, B> for A {
    fn intersect(&self, self_pose: &P, other: &B, other_pose: &P) -> bool {
        let self_pose = self.get_isometry3_cow(self_pose);
        let other_pose = other.get_isometry3_cow(other_pose);

        query::intersection_test(self_pose.as_ref(), &**self.shape(), other_pose.as_ref(), &**other.shape()).expect("error")
    }
}
impl<T: AD, P: O3DPose<T>, A: OShapeParryHierarchyTrait<T, P>, B: OShapeParryHierarchyTrait<T, P>> OBoundingSphereToBoundingSphereIntersectTrait<T, P, B> for A {
    fn bounding_sphere_to_bounding_sphere_intersect(&self, self_pose: &P, other: &B, other_pose: &P) -> bool {
        let self_pose = self.bounding_sphere().get_isometry3_cow(self_pose);
        let other_pose = other.bounding_sphere().get_isometry3_cow(other_pose);

        query::intersection_test(self_pose.as_ref(), &**self.bounding_sphere().shape(), other_pose.as_ref(), &**other.bounding_sphere().shape()).expect("error")
    }
}
impl<T: AD, P: O3DPose<T>, A: OShapeParryHierarchyTrait<T, P>, B: OShapeParryHierarchyTrait<T, P>> OOBBToOBBIntersectTrait<T, P, B> for A {
    fn obb_to_obb_intersect(&self, self_pose: &P, other: &B, other_pose: &P) -> bool {
        let self_pose = self.obb().get_isometry3_cow(self_pose);
        let other_pose = other.obb().get_isometry3_cow(other_pose);

        query::intersection_test(self_pose.as_ref(), &**self.obb().shape(), other_pose.as_ref(), &**other.obb().shape()).expect("error")
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait OShapeDisTrait<T: AD, P: O3DPose<T>, B> {
    fn distance(&self, self_pose: &P, other: &B, other_pose: &P) -> T;
}
pub trait OBoundingSphereToBoundingSphereDistanceTrait<T: AD, P: O3DPose<T>, B> {
    fn bounding_sphere_to_bounding_sphere_distance(&self, self_pose: &P, other: &B, other_pose: &P) -> T;
}
pub trait OOBBToOBBDistanceTrait<T: AD, P: O3DPose<T>, B> {
    fn obb_to_obb_distance(&self, self_pose: &P, other: &B, other_pose: &P) -> T;
}

impl<T: AD, P: O3DPose<T>, A: OShapeParryGenericTrait<T>, B: OShapeParryGenericTrait<T>> OShapeDisTrait<T, P, B> for A {
    fn distance(&self, self_pose: &P, other: &B, other_pose: &P) -> T {
        let self_pose = self.get_isometry3_cow(self_pose);
        let other_pose = other.get_isometry3_cow(other_pose);

        query::distance(self_pose.as_ref(), &**self.shape(), other_pose.as_ref(), &**other.shape()).expect("error")
    }
}
impl<T: AD, P: O3DPose<T>, A: OShapeParryHierarchyTrait<T, P>, B: OShapeParryHierarchyTrait<T, P>> OBoundingSphereToBoundingSphereDistanceTrait<T, P, B> for A {
    fn bounding_sphere_to_bounding_sphere_distance(&self, self_pose: &P, other: &B, other_pose: &P) -> T {
        let self_pose = self.bounding_sphere().get_isometry3_cow(self_pose);
        let other_pose = other.bounding_sphere().get_isometry3_cow(other_pose);

        query::distance(self_pose.as_ref(), &**self.bounding_sphere().shape(), other_pose.as_ref(), &**other.bounding_sphere().shape()).expect("error")
    }
}
impl<T: AD, P: O3DPose<T>, A: OShapeParryHierarchyTrait<T, P>, B: OShapeParryHierarchyTrait<T, P>> OOBBToOBBDistanceTrait<T, P, B> for A {
    fn obb_to_obb_distance(&self, self_pose: &P, other: &B, other_pose: &P) -> T {
        let self_pose = self.obb().get_isometry3_cow(self_pose);
        let other_pose = other.obb().get_isometry3_cow(other_pose);

        query::distance(self_pose.as_ref(), &**self.obb().shape(), other_pose.as_ref(), &**other.obb().shape()).expect("error")
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait OShapeContactTrait<T: AD, P: O3DPose<T>, B> {
    type Args : AsAny;
    type ContactOutput : OContactOutputTrait<T> + AsAny;

    fn contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> Self::ContactOutput;

    #[inline]
    fn distance_via_contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> T {
        let contact = self.contact(self_pose, other, other_pose, args);
        return contact.to_distance()
    }
}
pub trait OBoundingSphereToBoundingSphereContactTrait<T: AD, P: O3DPose<T>, B> {
    type Args : AsAny;

    type ContactOutput : OContactOutputTrait<T> + AsAny;

    fn bounding_sphere_to_bounding_sphere_contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> Self::ContactOutput;

    #[inline]
    fn bounding_sphere_to_bounding_sphere_distance_via_contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> T {
        let contact = self.bounding_sphere_to_bounding_sphere_contact(self_pose, other, other_pose, args);
        return contact.to_distance()
    }
}
pub trait OOBBToOBBContactTrait<T: AD, P: O3DPose<T>, B> {
    type Args : AsAny;

    type ContactOutput : OContactOutputTrait<T> + AsAny;

    fn obb_to_obb_contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> Self::ContactOutput;

    #[inline]
    fn obb_to_obb_distance_via_contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> T {
        let contact = self.obb_to_obb_contact(self_pose, other, other_pose, args);
        return contact.to_distance()
    }
}

impl<T: AD, P: O3DPose<T>, A: OShapeParryGenericTrait<T>, B: OShapeParryGenericTrait<T>> OShapeContactTrait<T, P, B> for A {
    type Args = T;
    type ContactOutput = OContactParry<T>;

    fn contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> Self::ContactOutput {
        let self_pose = self.get_isometry3_cow(self_pose);
        let other_pose = other.get_isometry3_cow(other_pose);

        OContactParry(query::contact(self_pose.as_ref(), &**self.shape(), other_pose.as_ref(), &**other.shape(), *args).expect("error"))
    }
}
impl<T: AD, P: O3DPose<T>, A: OShapeParryHierarchyTrait<T, P>, B: OShapeParryHierarchyTrait<T, P>> OBoundingSphereToBoundingSphereContactTrait<T, P, B> for A {
    type Args = T;
    type ContactOutput = OContactParry<T>;

    fn bounding_sphere_to_bounding_sphere_contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> Self::ContactOutput {
        let self_pose = self.bounding_sphere().get_isometry3_cow(self_pose);
        let other_pose = other.bounding_sphere().get_isometry3_cow(other_pose);

        OContactParry(query::contact(self_pose.as_ref(), &**self.bounding_sphere().shape(), other_pose.as_ref(), &**other.bounding_sphere().shape(), *args).expect("error"))
    }
}
impl<T: AD, P: O3DPose<T>, A: OShapeParryHierarchyTrait<T, P>, B: OShapeParryHierarchyTrait<T, P>> OOBBToOBBContactTrait<T, P, B> for A {
    type Args = T;
    type ContactOutput = OContactParry<T>;

    fn obb_to_obb_contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> Self::ContactOutput {
        let self_pose = self.obb().get_isometry3_cow(self_pose);
        let other_pose = other.obb().get_isometry3_cow(other_pose);

        OContactParry(query::contact(self_pose.as_ref(), &**self.obb().shape(), other_pose.as_ref(), &**other.obb().shape(), *args).expect("error"))
    }
}

pub trait OContactOutputTrait<T: AD> {
    fn to_distance(&self) -> T;
}

#[derive(Clone, Debug)]
pub struct OContactParry<T: AD>(pub Option<Contact<T>>);
impl<T: AD> OContactOutputTrait<T> for OContactParry<T> {
    #[inline]
    fn to_distance(&self) -> T {
        match &self.0 {
            None => { T::constant(f64::INFINITY) }
            Some(c) => { c.dist }
        }
    }
}
impl<T: AD> PartialEq for OContactParry<T> {
    fn eq(&self, other: &Self) -> bool {
        match (&self.0, &other.0) {
            (Some(a), Some(b)) => { a.dist.eq(&b.dist) }
            (Some(_), None) => { false }
            (None, Some(_)) => { false }
            (None, None) => { true }
        }
    }
}
impl<T: AD> PartialOrd for OContactParry<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match (&self.0, &other.0) {
            (Some(a), Some(b)) => { a.dist.partial_cmp(&b.dist) }
            (Some(_), None) => { Some(Ordering::Greater) }
            (None, Some(_)) => { Some(Ordering::Less) }
            (None, None) => { Some(Ordering::Equal) }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
*/