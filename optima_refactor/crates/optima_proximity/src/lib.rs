use std::borrow::Cow;
use std::cmp::Ordering;
use ad_trait::AD;
use as_any::{AsAny};
use parry_ad::na::Isometry3;
use parry_ad::query;
use parry_ad::query::Contact;
use parry_ad::shape::Shape;
use optima_3d_spatial::optima_3d_pose::O3DPose;

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct OShapeParry<T: AD> {
    shape: Box<dyn Shape<T>>
}
impl<T: AD> OShapeParry<T> {
    pub fn new<S: Shape<T>>(shape: S) -> Self {
        Self {
            shape: Box::new(shape),
        }
    }
}
impl<T: AD> OShapeParryTrait<T> for OShapeParry<T> {
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }

    #[inline]
    fn get_isometry3_cow<'a, P: O3DPose<T>>(&self, pose: &'a P) -> Cow<'a, Isometry3<T>> {
        pose.downcast_or_convert::<Isometry3<T>>()
    }
}

pub struct OShapeParryWithOffset<T: AD, P: O3DPose<T>> {
    shape: Box<dyn Shape<T>>,
    offset: P
}
impl<T: AD, P: O3DPose<T>> OShapeParryWithOffset<T, P> {
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
impl<T: AD, P: O3DPose<T>> OShapeParryTrait<T> for OShapeParryWithOffset<T, P> {
    #[inline]
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }

    #[inline]
    fn get_isometry3_cow<'a, P2: O3DPose<T>>(&self, pose: &'a P2) -> Cow<'a, Isometry3<T>> {
        let offset: Cow<P2> = self.offset.downcast_or_convert::<P2>();
        let pose = offset.mul(pose);
        let res = pose.downcast_or_convert::<Isometry3<T>>();
        Cow::Owned(res.into_owned())
    }
}

pub trait OShapeParryTrait<T: AD> {
    fn shape(&self) -> &Box<dyn Shape<T>>;
    fn get_isometry3_cow<'a, P: O3DPose<T>>(&self, pose: &'a P) -> Cow<'a, Isometry3<T>>;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait OPairwiseShapeIntersectTrait<T: AD, P: O3DPose<T>, B> {
    fn intersect(&self, self_pose: &P, other: &B, other_pose: &P) -> bool;
}

impl<T: AD, P: O3DPose<T>, A: OShapeParryTrait<T>, B: OShapeParryTrait<T>> OPairwiseShapeIntersectTrait<T, P, B> for A {
    fn intersect(&self, self_pose: &P, other: &B, other_pose: &P) -> bool {
        let self_pose = self.get_isometry3_cow(self_pose);
        let other_pose = other.get_isometry3_cow(other_pose);

        query::intersection_test(self_pose.as_ref(), &**self.shape(), other_pose.as_ref(), &**other.shape()).expect("error")
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait OPairwiseShapeDisTrait<T: AD, P: O3DPose<T>, B> {
    fn distance(&self, self_pose: &P, other: &B, other_pose: &P) -> T;
}

impl<T: AD, P: O3DPose<T>, A: OShapeParryTrait<T>, B: OShapeParryTrait<T>> OPairwiseShapeDisTrait<T, P, B> for A {
    fn distance(&self, self_pose: &P, other: &B, other_pose: &P) -> T {
        let self_pose = self.get_isometry3_cow(self_pose);
        let other_pose = other.get_isometry3_cow(other_pose);

        query::distance(self_pose.as_ref(), &**self.shape(), other_pose.as_ref(), &**other.shape()).expect("error")
    }
}

/*
impl<T: AD, P: O3DPose<T>> PairwiseShapeDisTrait<T, P, OShapeParry<T>> for OShapeParry<T> {
    fn distance(&self, self_pose: &P, other: &Self, other_pose: &P) -> T {
        let self_pose = self.get_isometry3_cow(self_pose);
        let other_pose = other.get_isometry3_cow(other_pose);

        query::distance(self_pose.as_ref(), &**self.shape(), other_pose.as_ref(), &**other.shape()).expect("error")
    }
}
impl<T: AD, P: O3DPose<T>> PairwiseShapeDisTrait<T, P, OShapeParryWithOffset<T, P>> for OShapeParryWithOffset<T, P> {
    fn distance(&self, self_pose: &P, other: &OShapeParryWithOffset<T, P>, other_pose: &P) -> T {
        let self_pose = self.compute_offsetted_pose(self_pose);
        let other_pose = other.compute_offsetted_pose(other_pose);

        let binding = self_pose.downcast_or_convert::<Isometry3<T>>();
        let self_pose = binding.as_ref();
        let binding = other_pose.downcast_or_convert::<Isometry3<T>>();
        let other_pose = binding.as_ref();

        query::distance(&self_pose, &**self.shape(), &other_pose, &**other.shape()).expect("error")
    }
}
impl<T: AD, P: O3DPose<T>> PairwiseShapeDisTrait<T, P, OShapeParryWithOffset<T, P>> for OShapeParry<T> {
    fn distance(&self, self_pose: &P, other: &OShapeParryWithOffset<T, P>, other_pose: &P) -> T {
        let binding = self_pose.downcast_or_convert::<Isometry3<T>>();
        let self_pose = binding.as_ref();

        let other_pose = other.compute_offsetted_pose(other_pose);
        let binding = other_pose.downcast_or_convert::<Isometry3<T>>();
        let other_pose = binding.as_ref();

        query::distance(&self_pose, &**self.shape(), &other_pose, &**other.shape()).expect("error")
    }
}
impl<T: AD, P: O3DPose<T>> PairwiseShapeDisTrait<T, P, OShapeParry<T>> for OShapeParryWithOffset<T, P> {
    fn distance(&self, self_pose: &P, other: &OShapeParry<T>, other_pose: &P) -> T {
        other.distance(other_pose, self, self_pose)
    }
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait OPairwiseShapeContactTrait<T: AD, P: O3DPose<T>, B> {
    type Args : AsAny;
    type ContactOutput : OContactOutputTrait<T> + AsAny;

    fn contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> Self::ContactOutput;
    #[inline]
    fn distance_via_contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> T {
        let contact = self.contact(self_pose, other, other_pose, args);
        return contact.to_distance()
    }
}

pub trait OContactOutputTrait<T: AD> {
    fn to_distance(&self) -> T;
}

impl<T: AD, P: O3DPose<T>, A: OShapeParryTrait<T>, B: OShapeParryTrait<T>> OPairwiseShapeContactTrait<T, P, B> for A {
    type Args = T;
    type ContactOutput = OContactParry<T>;

    fn contact(&self, self_pose: &P, other: &B, other_pose: &P, args: &Self::Args) -> Self::ContactOutput {
        let self_pose = self.get_isometry3_cow(self_pose);
        let other_pose = other.get_isometry3_cow(other_pose);

        OContactParry(query::contact(self_pose.as_ref(), &**self.shape(), other_pose.as_ref(), &**other.shape(), *args).expect("error"))
    }
}

/*
impl<T: AD, P: O3DPose<T>> PairwiseShapeContactTrait<T, P, OShapeParry<T>> for OShapeParry<T> {
    /// prediction.  If the distance between shapes is greater than this, the result will be None.
    type Args = T;
    type ContactOutput = OContactParry<T>;

    fn contact(&self, self_pose: &P, other: &OShapeParry<T>, other_pose: &P, args: &T) -> Self::ContactOutput {
        let self_pose = self_pose.downcast_or_convert::<Isometry3<T>>();
        let other_pose = other_pose.downcast_or_convert::<Isometry3<T>>();

        let self_pose = self_pose.as_ref();
        let other_pose = other_pose.as_ref();
        OContactParry(query::contact(self_pose, &**self.shape(), other_pose, &**other.shape(), *args).expect("error"))
    }
}
impl<T: AD, P: O3DPose<T>> PairwiseShapeContactTrait<T, P, OShapeParryWithOffset<T, P>> for OShapeParryWithOffset<T, P> {
    /// prediction.  If the distance between shapes is greater than this, the result will be None.
    type Args = T;
    type ContactOutput = OContactParry<T>;

    fn contact(&self, self_pose: &P, other: &OShapeParryWithOffset<T, P>, other_pose: &P, args: &T) -> Self::ContactOutput {
        let self_pose = self.compute_offsetted_pose(self_pose);
        let other_pose = other.compute_offsetted_pose(other_pose);

        let binding = self_pose.downcast_or_convert::<Isometry3<T>>();
        let self_pose = binding.as_ref();
        let binding = other_pose.downcast_or_convert::<Isometry3<T>>();
        let other_pose = binding.as_ref();

        OContactParry(query::contact(self_pose, &**self.shape(), other_pose, &**other.shape(), *args).expect("error"))
    }
}
impl<T: AD, P: O3DPose<T>> PairwiseShapeContactTrait<T, P, OShapeParryWithOffset<T, P>> for OShapeParry<T> {
    /// prediction.  If the distance between shapes is greater than this, the result will be None.
    type Args = T;
    type ContactOutput = OContactParry<T>;

    fn contact(&self, self_pose: &P, other: &OShapeParryWithOffset<T, P>, other_pose: &P, args: &T) -> Self::ContactOutput {
        let binding = self_pose.downcast_or_convert::<Isometry3<T>>();
        let self_pose = binding.as_ref();

        let other_pose = other.compute_offsetted_pose(other_pose);
        let binding = other_pose.downcast_or_convert::<Isometry3<T>>();
        let other_pose = binding.as_ref();

        OContactParry(query::contact(self_pose, &**self.shape(), other_pose, &**other.shape(), *args).expect("error"))
    }
}
impl<T: AD, P: O3DPose<T>> PairwiseShapeContactTrait<T, P, OShapeParry<T>> for OShapeParryWithOffset<T, P> {
    /// prediction.  If the distance between shapes is greater than this, the result will be None.
    type Args = T;
    type ContactOutput = OContactParry<T>;

    fn contact(&self, self_pose: &P, other: &OShapeParry<T>, other_pose: &P, args: &T) -> Self::ContactOutput {
        let binding = other_pose.downcast_or_convert::<Isometry3<T>>();
        let other_pose = binding.as_ref();

        let self_pose = self.compute_offsetted_pose(self_pose);
        let binding = self_pose.downcast_or_convert::<Isometry3<T>>();
        let self_pose = binding.as_ref();

        OContactParry(query::contact(self_pose, &**self.shape(), other_pose, &**other.shape(), *args).expect("error"))
    }
}
*/

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