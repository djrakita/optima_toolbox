use std::borrow::Cow;
use ad_trait::AD;
use parry_ad::na::{Isometry3, Point3, Vector3};
use parry_ad::shape::{Ball, Cuboid, Shape};
use optima_3d_mesh::OTriMesh;
use optima_3d_spatial::optima_3d_pose::O3DPose;
use optima_3d_spatial::optima_3d_vec::{O3DVec, O3DVecCategoryPoint3};

pub trait OShapeParryGenericTrait<T: AD> {
    fn shape(&self) -> &Box<dyn Shape<T>>;
}

pub trait OShapeParryCowTrait<T: AD> {
    fn get_isometry3_cow<'a, P: O3DPose<T>>(&self, pose: &'a P) -> Cow<'a, Isometry3<T>>;
}
pub trait OShapeParryHierarchyTrait<T: AD, P: O3DPose<T>> : OShapeParryGenericTrait<T> + OShapeParryCowTrait<T> {
    fn bounding_sphere(&self) -> &OShapeParryGenericWithOffset<T, P>;
    fn obb(&self) -> &OShapeParryGenericWithOffset<T, P>;
}

pub struct OShapeParry<T: AD, P: O3DPose<T>> {
    shape: Box<dyn Shape<T>>,
    bounding_sphere: OShapeParryGenericWithOffset<T, P>,
    obb: OShapeParryGenericWithOffset<T, P>,
    offset: Option<P>
}
impl<T: AD, P: O3DPose<T>> OShapeParry<T, P> {
    pub fn new<S: Shape<T>>(shape: S, offset: Option<P>) -> Self {
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
            offset
        }
    }
    pub fn new_from_convex_shape(trimesh: &OTriMesh, offset: Option<P>) -> Self {
        let points: Vec<Point3<T>> = trimesh.points().iter().map(|x| x.to_other_generic_category::<T, O3DVecCategoryPoint3>() ).collect();
        let convex_polyhedron = parry_ad::shape::ConvexPolyhedron::from_convex_hull(&points).expect("not a convex shape");
        Self::new(convex_polyhedron, offset)
    }
}
impl<T: AD, P: O3DPose<T>> OShapeParryGenericTrait<T> for OShapeParry<T, P> {
    #[inline(always)]
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }
}
impl<T: AD, P: O3DPose<T>> OShapeParryCowTrait<T> for OShapeParry<T, P> {
    #[inline(always)]
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
impl<T: AD, P: O3DPose<T>> OShapeParryHierarchyTrait<T, P> for OShapeParry<T, P> {
    #[inline(always)]
    fn bounding_sphere(&self) -> &OShapeParryGenericWithOffset<T, P> {
        &self.bounding_sphere
    }

    #[inline(always)]
    fn obb(&self) -> &OShapeParryGenericWithOffset<T, P> {
        &self.obb
    }
}

/*
pub struct OShapeParryGeneric<T: AD> {
    shape: Box<dyn Shape<T>>
}
/*
impl<T: AD> OShapeParryGeneric<T> {
    pub (crate) fn new<S: Shape<T>>(shape: S) -> Self {
        Self {
            shape: Box::new(shape)
        }
    }
}
*/
impl<T: AD> OShapeParryGenericTrait<T> for OShapeParryGeneric<T> {
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }
}
impl<T: AD> OShapeParryCowTrait<T> for OShapeParryGeneric<T> {
    #[inline]
    fn get_isometry3_cow<'a, P: O3DPose<T>>(&self, pose: &'a P) -> Cow<'a, Isometry3<T>> {
        pose.downcast_or_convert::<Isometry3<T>>()
    }
}
*/

pub struct OShapeParryGenericWithOffset<T: AD, P: O3DPose<T>> {
    shape: Box<dyn Shape<T>>,
    offset: P
}
impl<T: AD, P: O3DPose<T>> OShapeParryGenericWithOffset<T, P> {
    pub (crate) fn new<S: Shape<T>>(shape: S, offset: P) -> Self {
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
}
impl<T: AD, P: O3DPose<T>> OShapeParryCowTrait<T> for OShapeParryGenericWithOffset<T, P> {
    #[inline]
    fn get_isometry3_cow<'a, P2: O3DPose<T>>(&self, pose: &'a P2) -> Cow<'a, Isometry3<T>> {
        let offset: Cow<P2> = self.offset.downcast_or_convert::<P2>();
        let pose = pose.mul(offset.as_ref());
        let res = pose.downcast_or_convert::<Isometry3<T>>();
        Cow::Owned(res.into_owned())
    }
}

/*
pub struct OShapeParryHierarchy<T: AD, P: O3DPose<T>> {
    shape: Box<dyn Shape<T>>,
    bounding_sphere: OShapeParryGenericWithOffset<T, P>,
    obb: OShapeParryGenericWithOffset<T, P>
}
/*
impl<T: AD, P: O3DPose<T>> OShapeParryHierarchy<T, P> {
    pub (crate) fn new<S: Shape<T>>(shape: S) -> Self {
        let shape = Box::new(shape);
        let bounding_sphere = get_bounding_sphere_from_shape(&shape, &P::identity());
        let obb = get_obb_from_shape(&shape, &P::identity());

        Self {
            shape,
            bounding_sphere,
            obb
        }
    }
    pub (crate) fn new_from_convex_shape(trimesh: &OTriMesh) -> Self {
        let points: Vec<Point3<T>> = trimesh.points().iter().map(|x| x.to_other_generic_category::<T, O3DVecCategoryPoint3>() ).collect();
        let convex_polyhedron = parry_ad::shape::ConvexPolyhedron::from_convex_hull(&points).expect("not a convex shape");
        Self::new(convex_polyhedron)
    }
}
*/
impl<T: AD, P: O3DPose<T>> OShapeParryGenericTrait<T> for OShapeParryHierarchy<T, P> {
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }
}
impl<T: AD, P: O3DPose<T>> OShapeParryCowTrait<T> for OShapeParryHierarchy<T, P> {
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
*/
/*
pub struct OShapeParryHierarchyWithOffset<T: AD, P: O3DPose<T>> {
    shape: Box<dyn Shape<T>>,
    offset: P,
    bounding_sphere: OShapeParryGenericWithOffset<T, P>,
    obb: OShapeParryGenericWithOffset<T, P>
}
/*
impl<T: AD, P: O3DPose<T>> OShapeParryHierarchyWithOffset<T, P> {
    pub (crate) fn new<S: Shape<T>>(shape: S, offset: P) -> Self {
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
    pub (crate) fn new_from_convex_shape(trimesh: &OTriMesh, offset: P) -> Self {
        let points: Vec<Point3<T>> = trimesh.points().iter().map(|x| x.to_other_generic_category::<T, O3DVecCategoryPoint3>() ).collect();
        let convex_polyhedron = parry_ad::shape::ConvexPolyhedron::from_convex_hull(&points).expect("not a convex shape");
        Self::new(convex_polyhedron, offset)
    }
}
*/
impl<T: AD, P: O3DPose<T>> OShapeParryGenericTrait<T> for OShapeParryHierarchyWithOffset<T, P> {
    #[inline(always)]
    fn shape(&self) -> &Box<dyn Shape<T>> {
        &self.shape
    }
}
impl<T: AD, P: O3DPose<T>> OShapeParryCowTrait<T> for OShapeParryHierarchyWithOffset<T, P> {
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
*/

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