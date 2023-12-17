use std::borrow::Cow;
use std::fmt::Formatter;
use std::marker::PhantomData;
use std::ops::{Mul};
use std::time::{Instant};
use ad_trait::AD;
use parry_ad::na::{Isometry3, Point3, Vector3};
use parry_ad::shape::{Ball, ConvexPolyhedron, Cuboid, Shape, TypedShape};
use parry_ad::transformation::vhacd::{VHACD, VHACDParameters};
use serde::ser::SerializeTuple;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde::de::{SeqAccess, Visitor};
use optima_3d_mesh::{OTriMesh};
use optima_3d_spatial::optima_3d_pose::{O3DPose};
use optima_3d_spatial::optima_3d_vec::{O3DVec};
use optima_linalg::OVec;
use optima_sampling::SimpleSampler;
use serde_with::*;
use ad_trait::SerdeAD;
use optima_3d_spatial::optima_3d_pose::SerdeO3DPose;
use optima_file::path::OStemCellPath;
use crate::pair_queries::{ParryContactOutput, ParryDisMode, ParryDistanceOutput, ParryIntersectOutput, ParryOutputAuxData, ParryQryShapeType, ParryShapeRep};
use crate::shape_queries::{OShpQryContactTrait, OShpQryDistanceTrait, OShpQryIntersectTrait};

pub trait ShapeCategoryTrait {
    type ShapeType<T: AD, P: O3DPose<T>>;
}
impl ShapeCategoryTrait for () {
    type ShapeType<T: AD, P: O3DPose<T>> = ();
}

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
#[derive(Clone, Serialize, Deserialize)]
pub struct OParryShape<T: AD, P: O3DPose<T>> {
    #[serde(deserialize_with="OParryShpGenericHierarchy::<T, P>::deserialize")]
    pub (crate) base_shape: OParryShpGenericHierarchy<T, P>,
    #[serde(deserialize_with="Vec::<OParryShpGenericHierarchy::<T, P>>::deserialize")]
    pub (crate) convex_subcomponents: Vec<OParryShpGenericHierarchy<T, P>>
}
impl<T: AD, P: O3DPose<T>> OParryShape<T, P> {
    pub fn new<S: Shape<T>>(shape: S, offset: P) -> Self {
        Self::new_with_path_option(shape, offset, None)
    }
    pub fn new_with_path_option<S: Shape<T>>(shape: S, offset: P, path: Option<OStemCellPath>) -> Self {
        let is_convex = shape.is_convex();

        let base_shape = OParryShpGenericHierarchy::new(shape, offset, path);

        if is_convex {
            Self {
                base_shape: base_shape.clone(),
                convex_subcomponents: vec![base_shape.clone()],
            }
        } else {
            let convex_subcomponents = calculate_convex_subcomponent_shapes(base_shape.base_shape.shape(), 8);
            Self {
                base_shape,
                convex_subcomponents
            }
        }
    }
    pub fn new_convex_shape_from_mesh_paths(trimesh_path: OStemCellPath, offset: P, convex_subcomponents_paths: Option<Vec<OStemCellPath>>) -> Self {
        let trimesh = OTriMesh::try_to_get_trimesh_from_path(&trimesh_path).expect("error");

        let points = trimesh.points_to_point3s::<T>();
        // let indices = trimesh.indices_as_u32s();

        // let t = TriMesh::new(points, indices);
        let convex_polyhedron = ConvexPolyhedron::from_convex_hull(&points).expect("error");

        match &convex_subcomponents_paths {
            None => {
                Self::new_with_path_option(convex_polyhedron, offset, Some(trimesh_path.clone()))
            }
            Some(convex_subcomponents) => {
                // let convex_polyhedron = ConvexPolyhedron::from_convex_hull(&trimesh.to_convex_hull().points_to_point3s()).expect("error");

                let mut s = vec![];

                convex_subcomponents.iter().for_each(|x| {
                    let trimesh = OTriMesh::try_to_get_trimesh_from_path(x).expect("error");
                    let points = trimesh.points_to_point3s::<T>();
                    let convex_polyhedron_subcomponent = ConvexPolyhedron::from_convex_hull(&points).expect("error");
                    s.push(OParryShpGenericHierarchy::new(convex_polyhedron_subcomponent, offset.clone(), Some(x.clone())));
                });

                Self {
                    base_shape: OParryShpGenericHierarchy::new(convex_polyhedron, offset, Some(trimesh_path.clone())),
                    convex_subcomponents: s,
                }
            }
        }
    }
    pub fn new_convex_shape_from_trimesh(trimesh: OTriMesh, offset: P, convex_subcomponents: Option<Vec<OTriMesh>>) -> Self {
        let points = trimesh.points_to_point3s::<T>();
        // let indices = trimesh.indices_as_u32s();

        let t = ConvexPolyhedron::from_convex_hull(&points).expect("error");

        match &convex_subcomponents {
            None => {
                Self::new_with_path_option(t, offset, None)
            }
            Some(convex_subcomponents) => {
                let convex_polyhedron = ConvexPolyhedron::from_convex_hull(&trimesh.to_convex_hull().points_to_point3s()).expect("error");

               let mut s = vec![];

                convex_subcomponents.iter().for_each(|x| {
                    let points = x.points_to_point3s::<T>();
                    let convex_polyhedron = ConvexPolyhedron::from_convex_hull(&points).expect("error");
                    s.push(OParryShpGenericHierarchy::new(convex_polyhedron, offset.clone(), None));
                });

                Self {
                    base_shape: OParryShpGenericHierarchy::new(convex_polyhedron, offset, None),
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
    /*
    pub fn set_id(&mut self, id: u64) {
        self.id = id;
    }
    */
    pub fn resample_all_ids(&mut self) -> Vec<(u64, u64)> {
        let mut out = vec![];

        out.extend(self.base_shape.resample_ids());
        self.convex_subcomponents.iter_mut().for_each(|x| {
            out.extend(x.resample_ids());
        });

        out
    }
}
impl<T: AD, P: O3DPose<T>> OShpQryIntersectTrait<T, P, OParryShape<T, P>> for OParryShape<T, P> {
    type Args = (ParryQryShapeType, ParryShapeRep);
    type Output = ParryIntersectOutput;

    fn intersect(&self, other: &OParryShape<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {

        return match &args.0 {
            ParryQryShapeType::Standard => { self.base_shape().intersect(other.base_shape(), pose_a, pose_b, &args.1)  }
            /*
            ParryQryShapeType::AllConvexSubcomponents => {
                let start = Instant::now();
                let mut count = 0;
                for c1 in self.convex_subcomponents.iter() {
                    for c2 in other.convex_subcomponents.iter() {
                        count += 1;
                        if c1.intersect(&c2, pose_a, pose_b, &args.1).intersect {
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
            */
            ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx, shape_b_subcomponent_idx } => {
                let shape_a = self.convex_subcomponents.get(*shape_a_subcomponent_idx).expect(&format!("idx error: idx {}, {:?}", shape_a_subcomponent_idx, self.convex_subcomponents.len()));
                let shape_b = other.convex_subcomponents.get(*shape_b_subcomponent_idx).expect(&format!("idx error: idx {}, len {:?}", shape_b_subcomponent_idx, self.convex_subcomponents.len()));

                shape_a.intersect(shape_b, pose_a, pose_b, &args.1)
            }
        }
    }
}
impl<T: AD, P: O3DPose<T>> OShpQryDistanceTrait<T, P, OParryShape<T, P>> for OParryShape<T, P> {
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryDistanceOutput<T>;

    fn distance(&self, other: &OParryShape<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        match &args.1 {
            ParryQryShapeType::Standard => { self.base_shape().distance(other.base_shape(), pose_a, pose_b, &(args.0.clone(), args.2.clone(), args.3))  }
            /*
            ParryQryShapeType::AllConvexSubcomponents => {
                let start = Instant::now();
                let mut count = 0;
                let mut min_dis = T::constant(f64::MAX);
                for c1 in self.convex_subcomponents.iter() {
                    for c2 in other.convex_subcomponents.iter() {
                        count += 1;
                        let dis = c1.distance(&c2, pose_a, pose_b, &(args.0.clone(), args.2.clone()));
                        if dis.distance < min_dis { min_dis = dis.distance }
                    }
                }
                ParryDistanceOutput {
                    distance: min_dis,
                    aux_data: ParryOutputAuxData { num_queries: count, duration: start.elapsed() }
                }
            }
            */
            ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx, shape_b_subcomponent_idx } => {
                let shape_a = self.convex_subcomponents.get(*shape_a_subcomponent_idx).expect("idx error");
                let shape_b = other.convex_subcomponents.get(*shape_b_subcomponent_idx).expect("idx error");

                shape_a.distance(shape_b, pose_a, pose_b, &(args.0.clone(), args.2.clone(), args.3))
            }
        }
    }
}
impl<T: AD, P: O3DPose<T>> OShpQryContactTrait<T, P, OParryShape<T, P>> for OParryShape<T, P> {
    type Args = (T, ParryQryShapeType, ParryShapeRep, Option<T>);
    type Output = ParryContactOutput<T>;

    fn contact(&self, other: &OParryShape<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        return match &args.1 {
            ParryQryShapeType::Standard => {
                self.base_shape().contact(other.base_shape(), pose_a, pose_b, &(args.0.clone(), args.2.clone(), args.3))
            }
            /*
            ParryQryShapeType::AllConvexSubcomponents => {
                let start = Instant::now();
                let mut count = 0;
                let mut min_dis = None;
                for c1 in self.convex_subcomponents.iter() {
                    for c2 in other.convex_subcomponents.iter() {
                        count += 1;
                        let c = c1.contact(&c2, pose_a, pose_b, &(args.0.clone(), args.2.clone()));
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
            */
            ParryQryShapeType::ConvexSubcomponentsWithIdxs { shape_a_subcomponent_idx, shape_b_subcomponent_idx } => {
                let shape_a = self.convex_subcomponents.get(*shape_a_subcomponent_idx).expect("idx error");
                let shape_b = other.convex_subcomponents.get(*shape_b_subcomponent_idx).expect("idx error");

                shape_a.contact(shape_b, pose_a, pose_b, &(args.0.clone(), args.2.clone(), args.3))
            }
        }
    }
}

pub struct ShapeCategoryOParryShape;
impl ShapeCategoryTrait for ShapeCategoryOParryShape {
    type ShapeType<T: AD, P: O3DPose<T>> = OParryShape<T, P>;
}

/*
impl<T: AD, P: O3DPose<T>> OShpQryDistanceLowerBoundTrait<T, P, OParryShape<T, P>> for OParryShape<T, P> {
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep);
    type Output = ParryDistanceLowerBoundOutput<T>;

    fn distance_lower_bound(&self, other: &OParryShape<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let res = parry_shape_lower_and_upper_bound(self, other, pose_a, pose_b, &args.0, &args.1, &args.2);
        ParryDistanceLowerBoundOutput {
            distance_lower_bound: res.distance_lower_bound,
            aux_data: res.aux_data.clone()
        }
    }
}
impl<T: AD, P: O3DPose<T>> OShpQryDistanceUpperBoundTrait<T, P, OParryShape<T, P>> for OParryShape<T, P> {
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep);
    type Output = ParryDistanceUpperBoundOutput<T>;

    fn distance_upper_bound(&self, other: &OParryShape<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let res = parry_shape_lower_and_upper_bound(self, other, pose_a, pose_b, &args.0, &args.1, &args.2);
        ParryDistanceUpperBoundOutput {
            distance_upper_bound: res.distance_upper_bound,
            aux_data: res.aux_data.clone()
        }
    }
}
impl<T: AD, P: O3DPose<T>> OShpQryDistanceBoundsTrait<T, P, OParryShape<T, P>> for OParryShape<T, P> {
    type Args = (ParryDisMode, ParryQryShapeType, ParryShapeRep);
    type Output = ParryDistanceBoundsOutput<T>;

    fn distance_bounds(&self, other: &OParryShape<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        parry_shape_lower_and_upper_bound(self, other, pose_a, pose_b, &args.0, &args.1, &args.2)
    }
}
*/

#[serde_as]
#[derive(Clone, Serialize, Deserialize)]
pub struct OParryShpGenericHierarchy<T: AD, P: O3DPose<T>> {
    #[serde(deserialize_with="OParryShpGeneric::<T, P>::deserialize")]
    pub (crate) base_shape: OParryShpGeneric<T, P>,
    #[serde(deserialize_with="OParryShpGeneric::<T, P>::deserialize")]
    pub (crate) bounding_sphere: OParryShpGeneric<T, P>,
    #[serde_as(as = "SerdeAD<T>")]
    pub (crate) bounding_sphere_max_dis_error: T,
    #[serde(deserialize_with="OParryShpGeneric::<T, P>::deserialize")]
    pub (crate) obb: OParryShpGeneric<T, P>,
    #[serde_as(as = "SerdeAD<T>")]
    pub (crate) obb_max_dis_error: T
}
impl<T: AD, P: O3DPose<T>> OParryShpGenericHierarchy<T, P> {
    pub (crate) fn new<S: Shape<T>>(shape: S, offset: P, path: Option<OStemCellPath>) -> Self {
        Self::new_from_box(Box::new(shape), offset, path)
    }
    pub (crate) fn new_from_box<S: Shape<T>>(shape: Box<S>, offset: P, path: Option<OStemCellPath>) -> Self {
        let base_shape = OParryShpGeneric::new_from_box(shape, offset.clone(), path);
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
    pub fn resample_ids(&mut self) -> Vec<(u64, u64)> {
        let mut out = vec![];

        out.extend(self.base_shape.resample_ids());
        out.extend(self.obb.resample_ids());
        out.extend(self.bounding_sphere.resample_ids());

        out
    }
    #[inline(always)]
    pub fn id_from_shape_rep(&self, shape_rep: &ParryShapeRep) -> u64 {
        match shape_rep {
            ParryShapeRep::Full => { self.base_shape.id }
            ParryShapeRep::OBB => { self.obb.id }
            ParryShapeRep::BoundingSphere => { self.bounding_sphere.id }
        }
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
    type Args = (ParryDisMode, ParryShapeRep, Option<T>);
    type Output = ParryDistanceOutput<T>;

    fn distance(&self, other: &OParryShpGenericHierarchy<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        match &args.1 {
            ParryShapeRep::Full => { self.base_shape.distance(&other.base_shape, pose_a, pose_b, &(args.0.clone(), args.2)) }
            ParryShapeRep::OBB => { self.obb.distance(&other.obb, pose_a, pose_b, &(args.0.clone(), args.2)) }
            ParryShapeRep::BoundingSphere => { self.bounding_sphere.distance(&other.bounding_sphere, pose_a, pose_b, &(args.0.clone(), args.2)) }
        }
    }
}
impl<T: AD, P: O3DPose<T>> OShpQryContactTrait<T, P,OParryShpGenericHierarchy<T, P>> for OParryShpGenericHierarchy<T, P> {
    type Args = (T, ParryShapeRep, Option<T>);
    type Output = ParryContactOutput<T>;

    fn contact(&self, other: &OParryShpGenericHierarchy<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> ParryContactOutput<T> {
        match &args.1 {
            ParryShapeRep::Full => { self.base_shape.contact(&other.base_shape, pose_a, pose_b, &(args.0, args.2)) }
            ParryShapeRep::OBB => { self.obb.contact(&other.obb, pose_a, pose_b, &(args.0, args.2)) }
            ParryShapeRep::BoundingSphere => { self.bounding_sphere.contact(&other.bounding_sphere, pose_a, pose_b, &(args.0, args.2)) }
        }
    }
}

#[serde_as]
#[derive(Serialize, Deserialize)]
pub struct OParryShpGeneric<T: AD, P: O3DPose<T>> {
    pub (crate) id: u64,
    #[serde(deserialize_with="BoxedShape::<T>::deserialize")]
    pub (crate) shape: BoxedShape<T>,
    #[serde_as(as = "SerdeO3DPose<T, P>")]
    pub (crate) offset: P,
    #[serde_as(as = "SerdeAD<T>")]
    pub (crate) max_dis_from_origin_to_point_on_shape: T
}
impl<T: AD, P: O3DPose<T>> OParryShpGeneric<T, P> {
    pub fn new<S: Shape<T>>(shape: S, offset: P, path: Option<OStemCellPath>) -> Self {
        Self::new_from_box(Box::new(shape), offset, path)
    }
    pub (crate) fn new_from_box<S: Shape<T>>(shape: Box<S>, offset: P, path: Option<OStemCellPath>) -> Self {
        let max_dis_from_origin_to_point_on_shape = calculate_max_dis_from_origin_to_point_on_shape(&shape);
        Self {
            id: SimpleSampler::uniform_sample_u64((u64::MIN, u64::MAX), None),
            shape: BoxedShape {shape, path},
            offset,
            max_dis_from_origin_to_point_on_shape,
        }
    }
    #[inline(always)]
    pub fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape.shape
    }
    #[inline(always)]
    pub fn boxed_shape(&self) -> &BoxedShape<T> { &self.shape }
    #[inline(always)]
    pub fn offset(&self) -> &P {
        &self.offset
    }
    #[inline(always)]
    pub fn id(&self) -> u64 {
        self.id
    }
    pub fn resample_ids(&mut self) -> Vec<(u64, u64)> {
        let mut out = vec![];

        let sample = SimpleSampler::uniform_sample_u64((0,u64::MAX), None);
        out.push( (self.id, sample) );
        self.id = sample;

        out
    }
    #[inline(always)]
    pub fn max_dis_from_origin_to_point_on_shape(&self) -> T {
        self.max_dis_from_origin_to_point_on_shape
    }
}
impl<T: AD, P: O3DPose<T>> Clone for OParryShpGeneric<T, P> {
    fn clone(&self) -> Self {
        Self {
            id: self.id.clone(),
            shape: self.shape.clone(),
            offset: self.offset.clone(),
            max_dis_from_origin_to_point_on_shape: self.max_dis_from_origin_to_point_on_shape.clone(),
        }
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
        let offset: Cow<P2> = self.offset.o3dpose_downcast_or_convert::<P2>();
        let pose = pose.mul(offset.as_ref());
        let res = pose.o3dpose_downcast_or_convert::<Isometry3<T>>();
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
    type Args = (ParryDisMode, Option<T>);
    type Output = ParryDistanceOutput<T>;

    fn distance(&self, other: &OParryShpGeneric<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let start = Instant::now();
        match &args.0 {
            ParryDisMode::StandardDis => {
                let pose_a = self.get_isometry3_cow(pose_a);
                let pose_b = other.get_isometry3_cow(pose_b);
                let distance = parry_ad::query::distance(pose_a.as_ref(), &**self.shape(), pose_b.as_ref(), &**other.shape()).expect("error");

                let distance_wrt_average = match &args.1 {
                    None => { distance }
                    Some(a) => { distance / *a }
                };

                ParryDistanceOutput {
                    distance_wrt_average,
                    raw_distance: distance,
                    aux_data: ParryOutputAuxData { num_queries: 1, duration: start.elapsed() }
                }
            }
            ParryDisMode::ContactDis => {
                let c = self.contact(other, pose_a, pose_b, &(T::constant(f64::INFINITY), args.1));
                // let distance = c.signed_distance().expect(&format!("this should never be None.  {:?}, {:?}", pose_a, pose_b));

                /*
                let distance_wrt_average = match &args.1 {
                    None => { distance }
                    Some(a) => { distance / *a }
                };
                */

                ParryDistanceOutput {
                    distance_wrt_average: c.distance_wrt_average.unwrap(),
                    raw_distance: c.contact.unwrap().dist,
                    aux_data: ParryOutputAuxData { num_queries: 1, duration: start.elapsed() }
                }
            }
        }

    }
}
impl<T: AD, P: O3DPose<T>> OShpQryContactTrait<T, P, OParryShpGeneric<T, P>> for OParryShpGeneric<T, P> {
    type Args = (T, Option<T>);
    type Output = ParryContactOutput<T>;

    fn contact(&self, other: &OParryShpGeneric<T, P>, pose_a: &P, pose_b: &P, args: &Self::Args) -> Self::Output {
        let start = Instant::now();
        let pose_a = self.get_isometry3_cow(pose_a);
        let pose_b = other.get_isometry3_cow(pose_b);

        let contact = parry_ad::query::contact(pose_a.as_ref(), &**self.shape(), pose_b.as_ref(), &**other.shape(), args.0).expect("error");

        let distance_wrt_average = match &contact {
            None => { None }
            Some(c) => {
                match args.1 {
                    None => { Some(c.dist) }
                    Some(a) => { Some(c.dist / a) }
                }
            }
        };

        ParryContactOutput {
            distance_wrt_average,
            contact,
            aux_data: ParryOutputAuxData { num_queries: 1, duration: start.elapsed() },
        }
    }
}

pub struct BoxedShape<T: AD>{
    pub shape: Box<dyn Shape<T>>,
    pub path: Option<OStemCellPath>
}
impl<T: AD> Clone for BoxedShape<T> {
    fn clone(&self) -> Self {
        BoxedShape{shape: self.shape.clone_box(), path: self.path.clone() }
    }
}
impl<T: AD> Serialize for BoxedShape<T> {
    #[allow(unreachable_patterns)]
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
        let typed_shape = self.shape.as_typed_shape();
        let tuple = match &typed_shape {
            TypedShape::Ball(s) => {
                let mut tuple = serializer.serialize_tuple(2)?;
                tuple.serialize_element(&"ball".to_string())?;
                tuple.serialize_element(&s.radius.to_constant())?;
                tuple
            }
            TypedShape::Cuboid(s) => {
                let mut tuple = serializer.serialize_tuple(2)?;
                tuple.serialize_element(&"cuboid".to_string())?;
                tuple.serialize_element(&OVec::ovec_to_other_ad_type::<f64>(&s.half_extents))?;
                tuple
            }
            TypedShape::Capsule(_) => { panic!("shape not handled here") }
            TypedShape::Segment(_) => { panic!("shape not handled here") }
            TypedShape::Triangle(_) => { panic!("shape not handled here") }
            TypedShape::TriMesh(_) => { panic!("shape not handled here") }
            TypedShape::Polyline(_) => { panic!("shape not handled here") }
            TypedShape::HalfSpace(_) => { panic!("shape not handled here") }
            TypedShape::HeightField(_) => { panic!("shape not handled here") }
            TypedShape::Compound(_) => { panic!("shape not handled here") }
            TypedShape::ConvexPolyhedron(s) => {
                match &self.path {
                    None => {
                        let mut tuple = serializer.serialize_tuple(2)?;
                        tuple.serialize_element(&"convex_polyhedron_raw".to_string())?;
                        let trimesh = s.to_trimesh();
                        let points: Vec<[f64; 3]> = trimesh.0.iter().map(|x| [x.x.to_constant(), x.y.to_constant(), x.z.to_constant()]).collect();
                        // tuple.serialize_element(&(points, trimesh.1))?;
                        tuple.serialize_element(&points)?;
                        tuple
                    }
                    Some(path) => {
                        let mut tuple = serializer.serialize_tuple(2)?;
                        tuple.serialize_element(&"convex_polyhedron_from_file".to_string())?;
                        tuple.serialize_element(path)?;
                        tuple
                    }
                }
            }
            TypedShape::Cylinder(_) => { panic!("shape not handled here") }
            TypedShape::Cone(_) => { panic!("shape not handled here") }
            TypedShape::RoundCuboid(_) => { panic!("shape not handled here") }
            TypedShape::RoundTriangle(_) => { panic!("shape not handled here") }
            TypedShape::RoundCylinder(_) => { panic!("shape not handled here") }
            TypedShape::RoundCone(_) => { panic!("shape not handled here") }
            TypedShape::RoundConvexPolyhedron(_) => { panic!("shape not handled here") }
            TypedShape::Custom(_) => { panic!("shape not handled here") }
            _ => { panic!("shape not handled here") }
        };

        /*
        let translation_slice = value.translation().as_slice();
        let binding = value.rotation().scaled_axis_of_rotation();
        let rotation_slice = binding.as_slice();
        let slice_as_f64 = [
            translation_slice[0].to_constant(),
            translation_slice[1].to_constant(),
            translation_slice[2].to_constant(),
            rotation_slice[0].to_constant(),
            rotation_slice[1].to_constant(),
            rotation_slice[2].to_constant()
        ];
        let mut tuple = serializer.serialize_tuple(6)?;
        for element in &slice_as_f64 {
            tuple.serialize_element(element)?;
        }
        tuple.end()
        */
        // tuple.serialize_element(&self.path)?;

        tuple.end()
    }
}
impl<'de, T: AD> Deserialize<'de> for BoxedShape<T> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error> where D: Deserializer<'de> {
        let ret = deserializer.deserialize_tuple(2, BoxedShapeVisitor { 0: Default::default() });
        ret
    }
}

pub struct BoxedShapeVisitor<T: AD>(PhantomData<T>);
impl<'de, T: AD> Visitor<'de> for BoxedShapeVisitor<T> {
    type Value = BoxedShape<T>;

    fn expecting(&self, formatter: &mut Formatter) -> std::fmt::Result {
        formatter.write_str("a tuple")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error> where A: SeqAccess<'de> {
        let shape_type_str = seq.next_element::<String>().expect("error").expect("error");

        if shape_type_str == "ball" {
            let radius = seq.next_element::<f64>().expect("error").expect("error");
            // let _path = seq.next_element::<Option<OStemCellPath>>().expect("error").expect("error");
            let ret = Ok(BoxedShape {
                shape: Box::new(Ball::new(T::constant(radius))),
                path: None,
            });
            return ret;
        } else if shape_type_str == "cuboid" {
            let half_extents = seq.next_element::<[f64; 3]>().expect("error").expect("error");
            // let _path = seq.next_element::<Option<OStemCellPath>>().expect("error").expect("error");
            let half_extents = OVec::ovec_to_other_ad_type::<T>(&half_extents);
            return Ok(BoxedShape{
                shape: Box::new(Cuboid::new(Vector3::new(half_extents[0], half_extents[1], half_extents[2]))),
                path: None,
            })
        } else if shape_type_str == "convex_polyhedron_raw" {
            // let (points, _indices) = seq.next_element::<(Vec<[f64; 3]>, Vec<[u32; 3]>)>().expect("error").expect("error");
            let points = seq.next_element::<Vec<[f64; 3]>>().expect("error").expect("error");
            // let _path = seq.next_element::<Option<OStemCellPath>>().expect("error").expect("error");
            let points: Vec<Point3<T>> = points.iter().map(|x| Point3::new(T::constant(x[0]), T::constant(x[1]), T::constant(x[2]))).collect();
            // let convex_polyhedron = ConvexPolyhedron::from_convex_mesh(points, &indices).expect("error");
            let convex_polyhedron = ConvexPolyhedron::from_convex_hull(&points).expect("error");
            return Ok(BoxedShape{
                shape: Box::new(convex_polyhedron),
                path: None,
            })
        } else if shape_type_str == "convex_polyhedron_from_file" {
            let path = seq.next_element::<Option<OStemCellPath>>().expect("error").expect("error").unwrap();
            let trimesh = OTriMesh::try_to_get_trimesh_from_path(&path).expect("error");
            let convex_polyhedron = ConvexPolyhedron::from_convex_hull(&trimesh.points_to_point3s()).expect("error");
            return Ok(BoxedShape{
                shape: Box::new(convex_polyhedron),
                path: Some(path.clone()),
            })
        } else {
            panic!("shape not supported");
        }
    }
}

/*
#[allow(unreachable_patterns)]
pub fn boxed_shape_custom_serialize<S, T: AD>(value: &BoxedShape<T>, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {

    let typed_shape = value.0.as_typed_shape();
    let tuple = match &typed_shape {
        TypedShape::Ball(s) => {
            let mut tuple = serializer.serialize_tuple(2)?;
            tuple.serialize_element(&"ball".to_string())?;
            tuple.serialize_element(&s.radius.to_constant())?;
            tuple
        }
        TypedShape::Cuboid(s) => {
            let mut tuple = serializer.serialize_tuple(2)?;
            tuple.serialize_element(&"cuboid".to_string())?;
            tuple.serialize_element(&OVec::to_other_ad_type::<f64>(&s.half_extents))?;
            tuple
        }
        TypedShape::Capsule(_) => { todo!() }
        TypedShape::Segment(_) => { todo!() }
        TypedShape::Triangle(_) => { todo!() }
        TypedShape::TriMesh(_) => { todo!() }
        TypedShape::Polyline(_) => { todo!() }
        TypedShape::HalfSpace(_) => { todo!() }
        TypedShape::HeightField(_) => { todo!() }
        TypedShape::Compound(_) => { todo!() }
        TypedShape::ConvexPolyhedron(_) => { todo!() }
        TypedShape::Cylinder(_) => { todo!() }
        TypedShape::Cone(_) => { todo!() }
        TypedShape::RoundCuboid(_) => { panic!("shape not handled here") }
        TypedShape::RoundTriangle(_) => { panic!("shape not handled here") }
        TypedShape::RoundCylinder(_) => { panic!("shape not handled here") }
        TypedShape::RoundCone(_) => { panic!("shape not handled here") }
        TypedShape::RoundConvexPolyhedron(_) => { panic!("shape not handled here") }
        TypedShape::Custom(_) => { panic!("shape not handled here") }
        _ => { panic!("shape not handled here") }
    };

    /*
    let translation_slice = value.translation().as_slice();
    let binding = value.rotation().scaled_axis_of_rotation();
    let rotation_slice = binding.as_slice();
    let slice_as_f64 = [
        translation_slice[0].to_constant(),
        translation_slice[1].to_constant(),
        translation_slice[2].to_constant(),
        rotation_slice[0].to_constant(),
        rotation_slice[1].to_constant(),
        rotation_slice[2].to_constant()
    ];
    let mut tuple = serializer.serialize_tuple(6)?;
    for element in &slice_as_f64 {
        tuple.serialize_element(element)?;
    }
    tuple.end()
    */
    tuple.end()
}
*/

pub (crate) fn get_bounding_sphere_from_shape<T: AD, S: Shape<T> + ?Sized, P: O3DPose<T>>(shape: &Box<S>, offset: &P) -> OParryShpGeneric<T, P> {
    let bounding_sphere = shape.compute_local_bounding_sphere();
    let offset = offset.mul(&P::from_constructors(&bounding_sphere.center, &[T::zero();3]));
    let sphere = Ball::new(bounding_sphere.radius);
    OParryShpGeneric::new(sphere, offset, None)
}
pub (crate) fn get_obb_from_shape<T: AD, S: Shape<T> + ?Sized, P: O3DPose<T>>(shape: &Box<S>, offset: &P) -> OParryShpGeneric<T, P> {
    let aabb = shape.compute_local_aabb();
    let mins = aabb.mins;
    let maxs = aabb.maxs;
    let center = mins.o3dvec_add(&maxs).o3dvec_scalar_mul(T::constant(0.5));
    let offset = offset.mul(&P::from_constructors(&center, &[T::zero(); 3]));
    let half_x = (maxs[0] - mins[0]) * T::constant(0.5);
    let half_y = (maxs[1] - mins[1]) * T::constant(0.5);
    let half_z = (maxs[2] - mins[2]) * T::constant(0.5);
    let cuboid = Cuboid::new(Vector3::new(half_x, half_y, half_z));
    OParryShpGeneric::new(cuboid, offset, None)
}
pub (crate) fn calculate_max_dis_error_between_shape_and_bounding_shape<T: AD, S1: Shape<T> + ?Sized, S2: Shape<T> + ?Sized>(shape: &Box<S1>, bounding_shape: &Box<S2>) -> T {
    let ts = shape.as_typed_shape();

    let subdiv = 10;
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
    let num_samples = 7;
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
                let one_norm = sample.ovec_p_norm(&T::one());
                sample = sample.ovec_scalar_div(&one_norm);
                sample
            };
            let point = vertex0.mul(sample[0]).o3dvec_add(&vertex1.mul(sample[1])).o3dvec_add(&vertex2.mul(sample[2]));
            let projection = bounding_shape.project_local_point(&point, false);
            let dis = projection.point.o3dvec_sub(&point).norm();
            // assert!(dis <= T::constant(0.03) || projection.is_inside, "point: {}, projection: {}, dis: {}", point, projection.point, dis);
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
        out.push(OParryShpGenericHierarchy::new(convex_polyhedron, P::identity(), None));
    });

    out
}
pub (crate) fn calculate_max_dis_from_origin_to_point_on_shape<T: AD, S: Shape<T> + ?Sized>(shape: &Box<S>) -> T {
    let ts = shape.as_typed_shape();

    let subdiv = 50;
    let (points, _) = match &ts {
        TypedShape::Ball(shape) => { shape.to_trimesh(subdiv, subdiv) }
        TypedShape::Cuboid(shape) => { shape.to_trimesh() }
        TypedShape::Capsule(shape) => { shape.to_trimesh(subdiv, subdiv) }
        TypedShape::TriMesh(shape) => { (shape.vertices().clone(), shape.indices().clone()) }
        TypedShape::ConvexPolyhedron(shape) => { shape.to_trimesh() }
        TypedShape::Cylinder(shape) => { shape.to_trimesh(subdiv) }
        TypedShape::Cone(shape) => { shape.to_trimesh(subdiv) }
        _ => { panic!("shape type unsupported"); }
    };

    let mut max = T::constant(f64::MIN);
    points.iter().for_each(|x| {
        let norm = x.norm();
        if norm > max { max = norm; }
    });

    max
}

