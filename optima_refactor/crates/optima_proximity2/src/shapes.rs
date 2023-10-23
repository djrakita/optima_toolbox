use std::borrow::Cow;
use std::cmp::{Ordering};
use std::ops::{Mul};
use std::time::{Duration, Instant};
use ad_trait::AD;
use parry3d_f64::na::{Isometry3, Vector3};
use parry_ad::query::Contact;
use parry_ad::shape::{Ball, ConvexPolyhedron, Cuboid, Shape, TriMesh, TypedShape};
use parry_ad::transformation::vhacd::{VHACD, VHACDParameters};
use optima_3d_mesh::OTriMesh;
use optima_3d_spatial::optima_3d_pose::{O3DPose};
use optima_3d_spatial::optima_3d_vec::{O3DVec};
use optima_linalg::OVec;
use optima_sampling::SimpleSampler;
use crate::shape_queries::{ContactOutputTrait, DistanceOutputTrait, IntersectOutputTrait, OShpQryContactTrait, OShpQryDistanceTrait, OShpQryIntersectTrait};

pub trait OParryShpTrait<T: AD> {
    // fn shape(&self) -> &Box<dyn Shape<T>>;
    fn get_isometry3_cow<'a, P: O3DPose<T>>(&self, pose: &'a P) -> Cow<'a, Isometry3<T>>;
}

/*
pub struct OParryShp<T: AD, P: O3DPose<T>> {
    pub (crate) shape: Box<dyn Shape<T>>,
    pub (crate) bounding_sphere: OParryShpGeneric<T, P>,
    pub (crate) obb: OParryShpGeneric<T, P>,
    pub (crate) children: Option<Vec<Self>>,
    pub (crate) offset: Option<P>
}
impl<T: AD, P: O3DPose<T>> OParryShp<T, P> {
    pub fn new<S: Shape<T>>(shape: S, offset: Option<P>) -> Self {
        Self::new_internal(shape, None, offset)
    }
    pub (crate) fn new_internal<S: Shape<T>>(shape: S, children: Option<Vec<Self>>, offset: Option<P>) -> Self {
        let shape = Box::new(shape);
        let tmp_offset = match &offset {
            None => { P::identity() }
            Some(o) => { o.clone() }
        };
        let bounding_sphere = get_bounding_sphere_from_shape(&shape, &tmp_offset);
        let obb = get_obb_from_shape(&shape, &tmp_offset);

        Self {
            shape,
            bounding_sphere,
            obb,
            children,
            offset
        }
    }
    pub fn new_from_convex_shape(trimesh: &OTriMesh, children_trimeshes: Option<Vec<OTriMesh>>, offset: Option<P>) -> Self {
        let points: Vec<Point3<T>> = trimesh.points().iter().map(|x| x.to_other_generic_category::<T, O3DVecCategoryPoint3>() ).collect();
        let convex_polyhedron = parry_ad::shape::ConvexPolyhedron::from_convex_hull(&points).expect("not a convex shape");

        let children: Option<Vec<Self>> = match &children_trimeshes {
            None => { None }
            Some(c) => {
                let children = c.iter().map(|x| {
                    let points: Vec<Point3<T>> = x.points().iter().map(|y| y.to_other_generic_category::<T, O3DVecCategoryPoint3>() ).collect();
                    let convex_polyhedron = parry_ad::shape::ConvexPolyhedron::from_convex_hull(&points).expect("not a convex shape");
                    Self::new_internal(convex_polyhedron, None, offset.clone())
                }).collect();

                Some(children)
            }
        };

        Self::new_internal(convex_polyhedron, children, offset)
    }
}
impl<T: AD,  P: O3DPose<T>> OParryShpTrait<T> for OParryShp<T, P> {
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }

    fn get_isometry3_cow<'a, P2: O3DPose<T>>(&self, pose: &'a P2) -> Cow<'a, Isometry3<T>> {
        match &self.offset {
            None => { pose.downcast_or_convert::<Isometry3<T>>() }
            Some(offset) => {
                let offset: Cow<P2> = offset.downcast_or_convert::<P2>();
                let pose = pose.mul(offset.as_ref());
                let res = pose.downcast_or_convert::<Isometry3<T>>();
                Cow::Owned(res.into_owned())
            }
        }
    }
}
*/

/*
pub struct OParryShp2<T: AD, P: O3DPose<T>> {
    pub (crate) shape: Box<dyn Shape<T>>,
    pub (crate) bounding_sphere: OParryShpGeneric<T, P>,
    pub (crate) bounding_sphere_max_dis_error: T,
    pub (crate) obb: OParryShpGeneric<T, P>,
    pub (crate) obb_max_dis_error: T,
    pub (crate) convex_subcomponents: Option<Vec<OParryShpGeneric<T, P>>>,
    pub (crate) offset: Option<P>
}
impl<T: AD, P: O3DPose<T>> OParryShp2<T, P> {
    pub (crate) fn new_internal<S: Shape<T>>(shape: S, offset: Option<P>) -> Self {
        let shape = Box::new(shape);
        let tmp_offset = match &offset {
            None => { P::identity() }
            Some(o) => { o.clone() }
        };
        let bounding_sphere = get_bounding_sphere_from_shape(&shape, &tmp_offset);
        let obb = get_obb_from_shape(&shape, &tmp_offset);

        // let bounding_sphere_max_dis_error = calculate_max_dis_error_between_shape_and_bounding_shape(&shape, bounding_sphere.shape());
        // let obb_max_dis_error = calculate_max_dis_error_between_shape_and_bounding_shape(&shape, obb.shape());



        todo!()
    }
}
*/

#[derive(Clone)]
pub struct OParryShape<T: AD, P: O3DPose<T>> {
    pub (crate) base_shape: OParryShpGenericHierarchy<T, P>,
    pub (crate) convex_subcomponents: Vec<OParryShpGenericHierarchy<T, P>>
}
impl<T: AD, P: O3DPose<T>> OParryShape<T, P> {
    pub fn new<S: Shape<T>>(shape: S, offset: P) -> Self {
        let is_convex = shape.is_convex();

        let base_shape = OParryShpGenericHierarchy::new(shape, offset);

        if is_convex {
            Self {
                base_shape: base_shape.clone(),
                convex_subcomponents: vec![base_shape.clone()],
            }
        } else {
            let convex_subcomponents = calculate_convex_subcomponent_shapes(base_shape.base_shape.shape(), 6);
            Self {
                base_shape,
                convex_subcomponents
            }
        }
    }
    pub fn new_convex_shape_from_trimesh(trimesh: OTriMesh, offset: P, convex_subcomponents: Option<Vec<OTriMesh>>) -> Self {
        let points = trimesh.points_to_point3s::<T>();
        let indices = trimesh.indices_as_u32s();

        let t = TriMesh::new(points, indices);

        match &convex_subcomponents {
            None => {
                Self::new(t, offset)
            }
            Some(convex_subcomponents) => {
                let convex_polyhedron = ConvexPolyhedron::from_convex_hull(&trimesh.to_convex_hull().points_to_point3s()).expect("error");

               let mut s = vec![];

                convex_subcomponents.iter().for_each(|x| {
                    let points = x.points_to_point3s::<T>();
                    let convex_polyhedron = ConvexPolyhedron::from_convex_hull(&points).expect("error");
                    s.push(OParryShpGenericHierarchy::new(convex_polyhedron, offset.clone()));
                });

                Self {
                    base_shape: OParryShpGenericHierarchy::new(convex_polyhedron, offset),
                    convex_subcomponents: s,
                }

            }
        }
    }
    #[inline(always)]
    pub fn base_shape(&self) -> &OParryShpGenericHierarchy<T, P> {
        &self.base_shape
    }
    #[inline(always)]
    pub fn convex_subcomponents(&self) -> &Vec<OParryShpGenericHierarchy<T, P>> {
        &self.convex_subcomponents
    }
}

impl<T: AD, P: O3DPose<T>> OShpQryIntersectTrait<T, P, OParryShape<T, P>> for OParryShape<T, P> {
    type Args = ParryQryShapeType;
    type Output = ParryIntersectOutput;

    fn intersect(&self, other: &OParryShape<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {

        return match args {
            ParryQryShapeType::Standard(rep) => { self.base_shape().intersect(other.base_shape(), pose_a, pose_b, rep)  }
            ParryQryShapeType::ConvexSubcomponents(rep) => {
                let start = Instant::now();
                let mut count = 0;
                for c1 in self.convex_subcomponents.iter() {
                    for c2 in other.convex_subcomponents.iter() {
                        count += 1;
                        if c1.intersect(&c2, pose_a, pose_b, rep).intersect {
                            return ParryIntersectOutput {
                                intersect: true,
                                aux_data: ParryOutputAuxData { num_queries: count, duration: start.elapsed()}
                            };
                        }
                    }
                }
                ParryIntersectOutput {
                    intersect: false,
                    aux_data: ParryOutputAuxData { num_queries: count, duration: start.elapsed()}
                }
            }
            ParryQryShapeType::ConvexSubcomponentsWithIdxs { rep, shape_a_subcomponent_idx, shape_b_subcomponent_idx } => {
                let shape_a = self.convex_subcomponents.get(*shape_a_subcomponent_idx).expect("idx error");
                let shape_b = self.convex_subcomponents.get(*shape_b_subcomponent_idx).expect("idx error");

                shape_a.intersect(shape_b, pose_a, pose_b, rep)
            }
        }
    }
}
impl<T: AD, P: O3DPose<T>> OShpQryDistanceTrait<T, P, OParryShape<T, P>> for OParryShape<T, P> {
    type Args = (ParryDisMode, ParryQryShapeType);
    type Output = ParryDistanceOutput<T>;

    fn distance(&self, other: &OParryShape<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        match &args.1 {
            ParryQryShapeType::Standard(rep) => { self.base_shape().distance(other.base_shape(), pose_a, pose_b, &(args.0.clone(), rep.clone()))  }
            ParryQryShapeType::ConvexSubcomponents(rep) => {
                let start = Instant::now();
                let mut count = 0;
                let mut min_dis = T::constant(f64::MAX);
                for c1 in self.convex_subcomponents.iter() {
                    for c2 in other.convex_subcomponents.iter() {
                        count += 1;
                        let dis = c1.distance(&c2, pose_a, pose_b, &(args.0.clone(), rep.clone()));
                        if dis.distance < min_dis { min_dis = dis.distance }
                    }
                }
                ParryDistanceOutput {
                    distance: min_dis,
                    aux_data: ParryOutputAuxData { num_queries: count, duration: start.elapsed() }
                }
            }
            ParryQryShapeType::ConvexSubcomponentsWithIdxs { rep, shape_a_subcomponent_idx, shape_b_subcomponent_idx } => {
                let shape_a = self.convex_subcomponents.get(*shape_a_subcomponent_idx).expect("idx error");
                let shape_b = self.convex_subcomponents.get(*shape_b_subcomponent_idx).expect("idx error");

                shape_a.distance(shape_b, pose_a, pose_b, &(args.0.clone(), rep.clone()))
            }
        }
    }
}
impl<T: AD, P: O3DPose<T>> OShpQryContactTrait<T, P, OParryShape<T, P>> for OParryShape<T, P> {
    type Args = (T, ParryQryShapeType);
    type Output = ParryContactOutput<T>;

    fn contact(&self, other: &OParryShape<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        return match &args.1 {
            ParryQryShapeType::Standard(rep) => {
                self.base_shape().contact(other.base_shape(), pose_a, pose_b, &(args.0.clone(), rep.clone()))
            }
            ParryQryShapeType::ConvexSubcomponents(rep) => {
                let start = Instant::now();
                let mut count = 0;
                let mut min_dis = None;
                for c1 in self.convex_subcomponents.iter() {
                    for c2 in other.convex_subcomponents.iter() {
                        count += 1;
                        let c = c1.contact(&c2, pose_a, pose_b, &(args.0.clone(), rep.clone()));
                        if min_dis.is_none() { min_dis = Some(c); }
                        else if min_dis.as_ref().unwrap().gt(&c) { min_dis = Some(c); }
                    }
                }
                let min_dis = min_dis.unwrap();
                ParryContactOutput {
                    contact: min_dis.contact.clone(),
                    aux_data: ParryOutputAuxData { num_queries: count, duration: start.elapsed() },
                }
            }
            ParryQryShapeType::ConvexSubcomponentsWithIdxs { rep, shape_a_subcomponent_idx, shape_b_subcomponent_idx } => {
                let shape_a = self.convex_subcomponents.get(*shape_a_subcomponent_idx).expect("idx error");
                let shape_b = self.convex_subcomponents.get(*shape_b_subcomponent_idx).expect("idx error");

                shape_a.contact(shape_b, pose_a, pose_b, &(args.0.clone(), rep.clone()))
            }
        }
    }
}

#[derive(Clone)]
pub struct OParryShpGenericHierarchy<T: AD, P: O3DPose<T>> {
    pub (crate) base_shape: OParryShpGeneric<T, P>,
    pub (crate) bounding_sphere: OParryShpGeneric<T, P>,
    pub (crate) bounding_sphere_max_dis_error: T,
    pub (crate) obb: OParryShpGeneric<T, P>,
    pub (crate) obb_max_dis_error: T
}
impl<T: AD, P: O3DPose<T>> OParryShpGenericHierarchy<T, P> {
    pub (crate) fn new<S: Shape<T>>(shape: S, offset: P) -> Self {
        Self::new_from_box(Box::new(shape), offset)
    }
    pub (crate) fn new_from_box<S: Shape<T>>(shape: Box<S>, offset: P) -> Self {
        let base_shape = OParryShpGeneric::new_from_box(shape, offset.clone());
        let bounding_sphere = get_bounding_sphere_from_shape(base_shape.shape(), &offset);
        let bounding_sphere_max_dis_error = calculate_max_dis_error_between_shape_and_bounding_shape(base_shape.shape(), bounding_sphere.shape());
        let obb = get_obb_from_shape(base_shape.shape(), &offset);
        let obb_max_dis_error = calculate_max_dis_error_between_shape_and_bounding_shape(base_shape.shape(), obb.shape());

        Self {
            base_shape,
            bounding_sphere,
            bounding_sphere_max_dis_error,
            obb,
            obb_max_dis_error
        }
    }
    #[inline(always)]
    pub fn base_shape(&self) -> &OParryShpGeneric<T, P> {
        &self.base_shape
    }
    #[inline(always)]
    pub fn bounding_sphere(&self) -> &OParryShpGeneric<T, P> {
        &self.bounding_sphere
    }
    #[inline(always)]
    pub fn bounding_sphere_max_dis_error(&self) -> &T {
        &self.bounding_sphere_max_dis_error
    }
    #[inline(always)]
    pub fn obb(&self) -> &OParryShpGeneric<T, P> {
        &self.obb
    }
    #[inline(always)]
    pub fn obb_max_dis_error(&self) -> &T {
        &self.obb_max_dis_error
    }
}

impl<T: AD, P: O3DPose<T>> OShpQryIntersectTrait<T, P,OParryShpGenericHierarchy<T, P>> for OParryShpGenericHierarchy<T, P> {
    type Args = ParryShapeRep;
    type Output = ParryIntersectOutput;

    fn intersect(&self, other: &OParryShpGenericHierarchy<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        match args {
            ParryShapeRep::Full => { self.base_shape.intersect(&other.base_shape, pose_a, pose_b, &()) }
            ParryShapeRep::OBB => { self.obb.intersect(&other.obb, pose_a, pose_b, &()) }
            ParryShapeRep::BoundingSphere => { self.bounding_sphere.intersect(&other.bounding_sphere, pose_a, pose_b, &()) }
        }
    }
}
impl<T: AD, P: O3DPose<T>> OShpQryDistanceTrait<T, P,OParryShpGenericHierarchy<T, P>> for OParryShpGenericHierarchy<T, P> {
    type Args = (ParryDisMode, ParryShapeRep);
    type Output = ParryDistanceOutput<T>;

    fn distance(&self, other: &OParryShpGenericHierarchy<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        match &args.1 {
            ParryShapeRep::Full => { self.base_shape.distance(&other.base_shape, pose_a, pose_b, &args.0) }
            ParryShapeRep::OBB => { self.obb.distance(&other.obb, pose_a, pose_b, &args.0) }
            ParryShapeRep::BoundingSphere => { self.bounding_sphere.distance(&other.bounding_sphere, pose_a, pose_b, &args.0) }
        }
    }
}
impl<T: AD, P: O3DPose<T>> OShpQryContactTrait<T, P,OParryShpGenericHierarchy<T, P>> for OParryShpGenericHierarchy<T, P> {
    type Args = (T, ParryShapeRep);
    type Output = ParryContactOutput<T>;

    fn contact(&self, other: &OParryShpGenericHierarchy<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> ParryContactOutput<T> {
        match &args.1 {
            ParryShapeRep::Full => { self.base_shape.contact(&other.base_shape, pose_a, pose_b, &args.0) }
            ParryShapeRep::OBB => { self.obb.contact(&other.obb, pose_a, pose_b, &args.0) }
            ParryShapeRep::BoundingSphere => { self.bounding_sphere.contact(&other.bounding_sphere, pose_a, pose_b, &args.0) }
        }
    }
}

pub struct OParryShpGeneric<T: AD, P: O3DPose<T>> {
    pub (crate) shape: Box<dyn Shape<T>>,
    pub (crate) offset: P
}
impl<T: AD, P: O3DPose<T>> Clone for OParryShpGeneric<T, P> {
    fn clone(&self) -> Self {
        Self {
            shape: self.shape.clone_box(),
            offset: self.offset.clone(),
        }
    }
}
impl<T: AD, P: O3DPose<T>> OParryShpGeneric<T, P> {
    pub (crate) fn new<S: Shape<T>>(shape: S, offset: P) -> Self {
        Self::new_from_box(Box::new(shape), offset)
    }
    pub (crate) fn new_from_box<S: Shape<T>>(shape: Box<S>, offset: P) -> Self {
        Self {
            shape,
            offset
        }
    }
    #[inline(always)]
    pub fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }
    #[inline(always)]
    pub fn offset(&self) -> &P {
        &self.offset
    }
}
impl<T: AD, P: O3DPose<T>> OParryShpTrait<T> for OParryShpGeneric<T, P> {
    /*
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }
    */

    #[inline]
    fn get_isometry3_cow<'a, P2: O3DPose<T>>(&self, pose: &'a P2) -> Cow<'a, Isometry3<T>> {
        let offset: Cow<P2> = self.offset.downcast_or_convert::<P2>();
        let pose = pose.mul(offset.as_ref());
        let res = pose.downcast_or_convert::<Isometry3<T>>();
        Cow::Owned(res.into_owned())
    }
}

impl<T: AD, P: O3DPose<T>> OShpQryIntersectTrait<T, P, OParryShpGeneric<T, P>> for OParryShpGeneric<T, P> {
    type Args = ();
    type Output = ParryIntersectOutput;

    fn intersect(&self, other: &OParryShpGeneric<T, P>, pose_a: &P, pose_b: &P, _args: &Self::Args) -> Self::Output {
        let start = Instant::now();
        let pose_a = self.get_isometry3_cow(pose_a);
        let pose_b = other.get_isometry3_cow(pose_b);

        let intersect = parry_ad::query::intersection_test(pose_a.as_ref(), &**self.shape(), pose_b.as_ref(), &**other.shape()).expect("error");

        ParryIntersectOutput {
            intersect,
            aux_data: ParryOutputAuxData { num_queries: 1, duration: start.elapsed() },
        }
    }
}
impl<T: AD, P: O3DPose<T>> OShpQryDistanceTrait<T, P, OParryShpGeneric<T, P>> for OParryShpGeneric<T, P> {
    type Args = ParryDisMode;
    type Output = ParryDistanceOutput<T>;

    fn distance(&self, other: &OParryShpGeneric<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let start = Instant::now();
        match args {
            ParryDisMode::StandardDis => {
                let pose_a = self.get_isometry3_cow(pose_a);
                let pose_b = other.get_isometry3_cow(pose_b);
                let distance = parry_ad::query::distance(pose_a.as_ref(), &**self.shape(), pose_b.as_ref(), &**other.shape()).expect("error");

                ParryDistanceOutput {
                    distance,
                    aux_data: ParryOutputAuxData { num_queries: 1, duration: start.elapsed() }
                }
            }
            ParryDisMode::ContactDis => {
                let c = self.contact(other, pose_a, pose_b, &T::constant(f64::INFINITY));
                let distance = c.signed_distance().unwrap();
                ParryDistanceOutput {
                    distance,
                    aux_data: ParryOutputAuxData { num_queries: 1, duration: start.elapsed() }
                }
            }
        }

    }
}
impl<T: AD, P: O3DPose<T>> OShpQryContactTrait<T, P, OParryShpGeneric<T, P>> for OParryShpGeneric<T, P> {
    type Args = T;
    type Output = ParryContactOutput<T>;

    fn contact(&self, other: &OParryShpGeneric<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let start = Instant::now();
        let pose_a = self.get_isometry3_cow(pose_a);
        let pose_b = other.get_isometry3_cow(pose_b);

        let contact = parry_ad::query::contact(pose_a.as_ref(), &**self.shape(), pose_b.as_ref(), &**other.shape(), *args).expect("error");

        ParryContactOutput {
            contact,
            aux_data: ParryOutputAuxData { num_queries: 1, duration: start.elapsed() },
        }
    }
}

pub struct OParryBoundingSphere<T: AD, P: O3DPose<T>> {
    pub radius: T,
    pub offset: P
}
impl<T: AD, P: O3DPose<T>> Clone for OParryBoundingSphere<T, P> {
    fn clone(&self) -> Self {
        Self {
            radius: self.radius.clone(),
            offset: self.offset.clone(),
        }
    }
}
impl<T: AD, P: O3DPose<T>> OParryShpTrait<T> for OParryBoundingSphere<T, P> {
    /*
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }
    */

    #[inline]
    fn get_isometry3_cow<'a, P2: O3DPose<T>>(&self, pose: &'a P2) -> Cow<'a, Isometry3<T>> {
        let offset: Cow<P2> = self.offset.downcast_or_convert::<P2>();
        let pose = pose.mul(offset.as_ref());
        let res = pose.downcast_or_convert::<Isometry3<T>>();
        Cow::Owned(res.into_owned())
    }
}

impl<T: AD, P: O3DPose<T>> OShpQryIntersectTrait<T, P, OParryBoundingSphere<T, P>> for OParryBoundingSphere<T, P> {
    type Args = ();
    type Output = ParryIntersectOutput;

    fn intersect(&self, other: &OParryBoundingSphere<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let start = Instant::now();
        let pose_a = self.get_isometry3_cow(pose_a);
        let pose_b = self.get_isometry3_cow(pose_b);

        let dis = (&pose_a.translation.vector - &pose_b.translation.vector).norm();
        let thresh = self.radius + other.radius;

        let intersect = dis < thresh;
        ParryIntersectOutput {
            intersect,
            aux_data: ParryOutputAuxData { num_queries: 1, duration: start.elapsed() },
        }
    }
}

pub (crate) fn get_bounding_sphere_from_shape<T: AD, S: Shape<T> + ?Sized, P: O3DPose<T>>(shape: &Box<S>, offset: &P) -> OParryShpGeneric<T, P> {
    let bounding_sphere = shape.compute_local_bounding_sphere();
    let offset = offset.mul(&P::from_constructors(&bounding_sphere.center, &[T::zero();3]));
    let sphere = Ball::new(bounding_sphere.radius);
    OParryShpGeneric::new(sphere, offset)
}
pub (crate) fn get_obb_from_shape<T: AD, S: Shape<T> + ?Sized, P: O3DPose<T>>(shape: &Box<S>, offset: &P) -> OParryShpGeneric<T, P> {
    let aabb = shape.compute_local_aabb();
    let mins = aabb.mins;
    let maxs = aabb.maxs;
    let center = mins.add(&maxs).scalar_mul_o3dvec(T::constant(0.5));
    let offset = offset.mul(&P::from_constructors(&center, &[T::zero(); 3]));
    let half_x = (maxs[0] - mins[0]) * T::constant(0.5);
    let half_y = (maxs[1] - mins[1]) * T::constant(0.5);
    let half_z = (maxs[2] - mins[2]) * T::constant(0.5);
    let cuboid = Cuboid::new(Vector3::new(half_x, half_y, half_z));
    OParryShpGeneric::new(cuboid, offset)
}
pub (crate) fn calculate_max_dis_error_between_shape_and_bounding_shape<T: AD, S1: Shape<T> + ?Sized, S2: Shape<T> + ?Sized>(shape: &Box<S1>, bounding_shape: &Box<S2>) -> T {
    let ts = shape.as_typed_shape();

    let subdiv = 50;
    let (vertices, indices) = match &ts {
        TypedShape::Ball(shape) => { shape.to_trimesh(subdiv, subdiv) }
        TypedShape::Cuboid(shape) => { shape.to_trimesh() }
        TypedShape::Capsule(shape) => { shape.to_trimesh(subdiv, subdiv) }
        TypedShape::TriMesh(shape) => { (shape.vertices().clone(), shape.indices().clone()) }
        TypedShape::ConvexPolyhedron(shape) => { shape.to_trimesh() }
        TypedShape::Cylinder(shape) => { shape.to_trimesh(subdiv) }
        TypedShape::Cone(shape) => { shape.to_trimesh(subdiv) }
        _ => { panic!("shape type unsupported"); }
    };

    let mut max_dis = T::zero();
    /*
    vertices.iter().zip(indices.iter()).for_each(|(point, idx)| {
        let projection = bounding_shape.project_local_point(x, false);
        let dis = projection.point.sub(&x).norm();
        assert!(dis == T::zero() || projection.is_inside, "x: {}, projection: {}", x, projection.point);
        if dis > max_dis { max_dis = dis }
    });
    */
    let num_samples = 20;
    indices.iter().for_each(|x| {
        let idx0 = x[0] as usize;
        let idx1 = x[1] as usize;
        let idx2 = x[2] as usize;

        let vertex0 = &vertices[idx0];
        let vertex1 = &vertices[idx1];
        let vertex2 = &vertices[idx2];

        for i in 0..num_samples {
            let sample = if i == 0 {
                vec![T::constant(0.3333333); 3]
            } else if i == 1 {
                vec![T::constant(1.0), T::zero(), T::zero()]
            } else if i == 2 {
                vec![T::zero(), T::constant(1.0), T::zero()]
            } else if i == 3 {
                vec![T::zero(), T::zero(), T::constant(1.0)]
            } else if i == 4 {
                vec![T::constant(0.5), T::constant(0.5), T::zero()]
            } else if i == 5 {
                vec![T::constant(0.5), T::zero(), T::constant(0.5)]
            } else if i == 6 {
                vec![T::zero(), T::constant(0.5), T::constant(0.5)]
            } else {
                let mut sample = SimpleSampler::uniform_samples(&vec![(T::constant(0.0), T::constant(1.0)); 3], None);
                let one_norm = sample.lp_norm(&T::one());
                sample = sample.scalar_div(&one_norm);
                sample
            };
            let point = vertex0.mul(sample[0]).add(&vertex1.mul(sample[1])).add(&vertex2.mul(sample[2]));
            let projection = bounding_shape.project_local_point(&point, false);
            let dis = projection.point.sub(&point).norm();
            assert!(dis <= T::constant(0.000001) || projection.is_inside, "point: {}, projection: {}", point, projection.point);
            if dis > max_dis { max_dis = dis }
        }
    });

    max_dis
}
pub (crate) fn calculate_convex_subcomponent_shapes<T: AD, S: Shape<T> + ?Sized, P: O3DPose<T>>(shape: &Box<S>, max_convex_hulls: u32) -> Vec<OParryShpGenericHierarchy<T, P>> {
    let ts = shape.as_typed_shape();

    let subdiv = 50;
    let (points, indices) = match &ts {
        TypedShape::Ball(shape) => { shape.to_trimesh(subdiv, subdiv) }
        TypedShape::Cuboid(shape) => { shape.to_trimesh() }
        TypedShape::Capsule(shape) => { shape.to_trimesh(subdiv, subdiv) }
        TypedShape::TriMesh(shape) => { (shape.vertices().clone(), shape.indices().clone()) }
        TypedShape::ConvexPolyhedron(shape) => { shape.to_trimesh() }
        TypedShape::Cylinder(shape) => { shape.to_trimesh(subdiv) }
        TypedShape::Cone(shape) => { shape.to_trimesh(subdiv) }
        _ => { panic!("shape type unsupported"); }
    };

    let params = VHACDParameters {
        max_convex_hulls,
        resolution: 64,
        ..Default::default()
    };
    let res = VHACD::decompose(&params, &points, &indices, true);
    let convex_hulls = res.compute_convex_hulls(5);

    let mut out = vec![];

    convex_hulls.iter().for_each(|(points, _)| {
        let convex_polyhedron = ConvexPolyhedron::from_convex_hull(points).expect("error");
        out.push(OParryShpGenericHierarchy::new(convex_polyhedron, P::identity()));
    });

    out
}

#[derive(Clone, Debug)]
pub enum ParryDisMode {
    StandardDis, ContactDis
}

#[derive(Clone, Debug)]
pub enum ParryQryShapeType {
    Standard(ParryShapeRep), ConvexSubcomponents(ParryShapeRep), ConvexSubcomponentsWithIdxs { rep: ParryShapeRep, shape_a_subcomponent_idx: usize, shape_b_subcomponent_idx: usize }
}
impl ParryQryShapeType {
    pub fn new_standard(rep: ParryShapeRep) -> Self {
        Self::Standard(rep)
    }
    pub fn new_cvx_subs(rep: ParryShapeRep) -> Self {
        Self::ConvexSubcomponents(rep)
    }
    pub fn new_cvx_subs_w_idxs(idx0: usize, idx1: usize, rep: ParryShapeRep) -> Self {
        Self::ConvexSubcomponentsWithIdxs { rep, shape_a_subcomponent_idx: idx0, shape_b_subcomponent_idx: idx1 }
    }
}

#[derive(Clone, Debug)]
pub enum ParryShapeRep {
    Full, OBB, BoundingSphere
}

#[derive(Clone, Debug)]
pub struct ParryIntersectOutput {
    intersect: bool,
    aux_data: ParryOutputAuxData
}
impl ParryIntersectOutput {
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl PartialEq for ParryIntersectOutput {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.intersect.eq(&other.intersect)
    }
}
impl PartialOrd for ParryIntersectOutput {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        if self.intersect.eq(&other.intersect) { Some(Ordering::Equal) }
        else if self.intersect && !other.intersect { Some(Ordering::Less) }
        else { Some(Ordering::Greater) }
    }
}
impl IntersectOutputTrait for ParryIntersectOutput {
    #[inline(always)]
    fn intersect(&self) -> bool {
        self.intersect
    }
}

#[derive(Clone, Debug)]
pub struct ParryDistanceOutput<T: AD> {
    distance: T,
    aux_data: ParryOutputAuxData
}
impl<T: AD> ParryDistanceOutput<T> {
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl<T: AD> PartialEq for ParryDistanceOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        self.distance.eq(&other.distance)
    }
}
impl<T: AD> PartialOrd for ParryDistanceOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.distance.partial_cmp(&other.distance)
    }
}
impl<T: AD> DistanceOutputTrait<T> for ParryDistanceOutput<T> {
    #[inline(always)]
    fn distance(&self) -> T {
        self.distance
    }
}

#[derive(Clone, Debug)]
pub struct ParryContactOutput<T: AD> {
    contact: Option<Contact<T>>,
    aux_data: ParryOutputAuxData
}
impl<T: AD> ParryContactOutput<T> {
    #[inline(always)]
    pub fn contact(&self) -> Option<Contact<T>> {
        self.contact
    }
    #[inline(always)]
    pub fn aux_data(&self) -> &ParryOutputAuxData {
        &self.aux_data
    }
}
impl<T: AD> PartialEq for ParryContactOutput<T> {
    #[inline(always)]
    fn eq(&self, other: &Self) -> bool {
        match (&self.contact, &other.contact) {
            (Some(a), Some(b)) => { a.dist.eq(&b.dist) }
            (Some(_), None) => { false }
            (None, Some(_)) => { false }
            (None, None) => { true }
        }
    }
}
impl<T: AD> PartialOrd for ParryContactOutput<T> {
    #[inline(always)]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        match (&self.contact, &other.contact) {
            (Some(a), Some(b)) => { a.dist.partial_cmp(&b.dist) }
            (Some(_), None) => { Some(Ordering::Greater) }
            (None, Some(_)) => { Some(Ordering::Less) }
            (None, None) => { Some(Ordering::Equal) }
        }
    }
}
impl<T: AD> ContactOutputTrait<T> for ParryContactOutput<T> {
    #[inline(always)]
    fn signed_distance(&self) -> Option<T> {
        match &self.contact {
            None => { None }
            Some(c) => { Some(c.dist) }
        }
    }
}

#[derive(Clone, Debug)]
pub struct ParryOutputAuxData {
    num_queries: usize,
    duration: Duration
}
impl ParryOutputAuxData {
    #[inline(always)]
    pub fn num_queries(&self) -> usize {
        self.num_queries
    }
    #[inline(always)]
    pub fn duration(&self) -> Duration {
        self.duration
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
pub struct OParryShp<T: AD> {
    shape: Box<dyn Shape<T>>
}
impl<T: AD> OParryShp<T> {
    pub fn new<S: Shape<T>>(shape: S) -> Self {
        Self {
            shape: Box::new(shape),
        }
    }
}
impl<T: AD> OParryShpTrait<T> for OParryShp<T> {
    #[inline(always)]
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }

    #[inline(always)]
    fn get_isometry3_cow<'a, P: O3DPose<T>>(&self, pose: &'a P) -> Cow<'a, Isometry3<T>> {
        pose.downcast_or_convert::<Isometry3<T>>()
    }
}
impl<T: AD> OParryShpImplTrait<T> for OParryShp<T> {  }

pub struct OParryShpWithOffset<T: AD, C: O3DPoseCategoryTrait> {
    shape: Box<dyn Shape<T>>,
    offset: C::P<T>
}
impl<T: AD, C: O3DPoseCategoryTrait> OParryShpWithOffset<T, C> {
    pub fn new<S: Shape<T>>(shape: S, offset: C::P<T>) -> Self {
        Self {
            shape: Box::new(shape),
            offset
        }
    }
}
impl<T: AD, C: O3DPoseCategoryTrait> OParryShpTrait<T> for OParryShpWithOffset<T, C> {
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }

    fn get_isometry3_cow<'a, P: O3DPose<T>>(&self, pose: &'a P) -> Cow<'a, Isometry3<T>> {
        let offset: Cow<P> = self.offset.downcast_or_convert::<P>();
        let pose = pose.mul(offset.as_ref());
        let res = pose.downcast_or_convert::<Isometry3<T>>();
        Cow::Owned(res.into_owned())
    }
}
impl<T: AD, C: O3DPoseCategoryTrait> OParryShpImplTrait<T> for OParryShpWithOffset<T, C> {  }

impl<T: AD, P: O3DPose<T>, S1: OParryShpImplTrait<T> + 'static, S2: OParryShpImplTrait<T> + 'static> OShpQryIntersectTrait<T, P, S1> for S2 {
    type Args = ();

    fn intersect(&self, other: &S1, pose_a: &P, pose_b: &P, _args: &Self::Args) -> bool {
        let pose_a = self.get_isometry3_cow(pose_a);
        let pose_b = other.get_isometry3_cow(pose_b);

        parry_ad::query::intersection_test(pose_a.as_ref(), &**self.shape(), pose_b.as_ref(), &**other.shape()).expect("error")
    }
}
*/
