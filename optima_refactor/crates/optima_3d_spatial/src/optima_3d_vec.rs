use std::any::Any;
use std::fmt;
use std::fmt::{Debug};
use std::marker::PhantomData;
use ad_trait::{AD};
use nalgebra::{Point3, Vector3};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde::de::{SeqAccess, Visitor};
use serde::ser::SerializeTuple;
use serde_with::{DeserializeAs, SerializeAs};

#[derive(Clone, Debug, Copy, Eq, PartialEq)]
pub enum O3DVecType {
    Arr, Vec, NalgebraVector3, NalgebraPoint3
}

pub trait O3DVecCategoryTrait :
    Clone + Debug + Serialize + for<'a> Deserialize<'a> + Send + Sync
{
    type V<T: AD>: O3DVec<T>;
}

pub trait O3DVec<T: AD> :
    Clone + Debug + Serialize + for<'a> Deserialize<'a> + Send + Sync
{
    type Category: O3DVecCategoryTrait;

    fn as_any(&self) -> &dyn Any;
    fn type_identifier() -> O3DVecType;
    fn x(&self) -> T;
    fn y(&self) -> T;
    fn z(&self) -> T;
    fn from_slice(slice: &[T]) -> Self;
    fn as_slice(&self) -> &[T];
    fn to_arr(&self) -> [T; 3] { [self.x(), self.y(), self.z()] }
    fn add(&self, other: &Self) -> Self;
    fn sub(&self, other: &Self) -> Self;
    fn scalar_mul_o3dvec(&self, scalar: T) -> Self;
    fn norm(&self) -> T;
    fn dot(&self, other: &Self) -> T;
    fn cross(&self, other: &Self) -> Self;
    fn dis(&self, other: &Self) -> T;
    fn to_other_generic_category<T2: AD, C: O3DVecCategoryTrait>(&self) -> C::V<T2> {
        let x = self.x().to_constant();
        let y = self.y().to_constant();
        let z = self.z().to_constant();
        C::V::from_slice(&[T2::constant(x), T2::constant(y), T2::constant(z)])
    }
    fn to_other_ad_type<T2: AD>(&self) -> <Self::Category as O3DVecCategoryTrait>::V<T2> {
        self.to_other_generic_category::<T2, Self::Category>()
    }
}

impl<T: AD> O3DVec<T> for [T; 3] {
    type Category = O3DVecCategoryArr;

    #[inline(always)]
    fn as_any(&self) -> &dyn Any {
        self
    }

    #[inline(always)]
    fn type_identifier() -> O3DVecType { O3DVecType::Arr }

    #[inline(always)]
    fn x(&self) -> T {
        self[0]
    }

    #[inline(always)]
    fn y(&self) -> T {
        self[1]
    }

    #[inline(always)]
    fn z(&self) -> T {
        self[2]
    }

    #[inline(always)]
    fn from_slice(slice: &[T]) -> Self {
        [slice[0], slice[1], slice[2]]
    }

    #[inline(always)]
    fn as_slice(&self) -> &[T] {
        self
    }

    fn to_arr(&self) -> [T; 3] {
        *self
    }

    #[inline]
    fn add(&self, other: &Self) -> Self {
        [ self[0] + other[0], self[1] + other[1], self[2] + other[2] ]
    }

    #[inline]
    fn sub(&self, other: &Self) -> Self {
        [ self[0] - other[0], self[1] - other[1], self[2] - other[2] ]
    }

    #[inline]
    fn scalar_mul_o3dvec(&self, scalar: T) -> Self {
        [ scalar * self[0], scalar * self[1], scalar * self[2] ]
    }

    #[inline]
    fn norm(&self) -> T {
        (self[0].powi(2) + self[1].powi(2) + self[2].powi(2)).sqrt()
    }

    #[inline]
    fn dot(&self, other: &Self) -> T {
        self[0]*other[0] + self[1]*other[1] + self[2]*other[2]
    }

    #[inline]
    fn cross(&self, other: &Self) -> Self {
        [
            self[1] * other[2] - self[2] * other[1],
            self[2] * other[0] - self[0] * other[2],
            self[0] * other[1] - self[1] * other[0]
        ]
    }

    #[inline]
    fn dis(&self, other: &Self) -> T {
        self.sub(other).norm()
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct O3DVecCategoryArr;
impl O3DVecCategoryTrait for O3DVecCategoryArr {
    type V<T: AD> = [T; 3];
}

impl<T: AD> O3DVec<T> for Vec<T> {
    type Category = O3DVecCategoryVec;

    #[inline(always)]
    fn as_any(&self) -> &dyn Any {
        self
    }

    #[inline(always)]
    fn type_identifier() -> O3DVecType { O3DVecType::Vec }

    #[inline(always)]
    fn x(&self) -> T {
        self[0]
    }

    #[inline(always)]
    fn y(&self) -> T {
        self[1]
    }

    #[inline(always)]
    fn z(&self) -> T {
        self[2]
    }

    #[inline(always)]
    fn from_slice(slice: &[T]) -> Self {
        vec![slice[0], slice[1], slice[2]]
    }

    #[inline(always)]
    fn as_slice(&self) -> &[T] {
        self
    }

    #[inline]
    fn add(&self, other: &Self) -> Self {
        vec![ self[0] + other[0], self[1] + other[1], self[2] + other[2] ]
    }

    #[inline]
    fn sub(&self, other: &Self) -> Self {
        vec![ self[0] - other[0], self[1] - other[1], self[2] - other[2] ]
    }

    #[inline]
    fn scalar_mul_o3dvec(&self, scalar: T) -> Self {
        vec![ scalar * self[0], scalar * self[1], scalar * self[2] ]
    }

    #[inline]
    fn norm(&self) -> T {
        (self[0].powi(2) + self[1].powi(2) + self[2].powi(2)).sqrt()
    }

    fn dot(&self, other: &Self) -> T {
        self[0]*other[0] + self[1]*other[1] + self[2]*other[2]
    }

    fn cross(&self, other: &Self) -> Self {
        vec![
            self[1] * other[2] - self[2] * other[1],
            self[2] * other[0] - self[0] * other[2],
            self[0] * other[1] - self[1] * other[0]
        ]
    }

    #[inline]
    fn dis(&self, other: &Self) -> T {
        self.sub(other).norm()
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct O3DVecCategoryVec;
impl O3DVecCategoryTrait for O3DVecCategoryVec {
    type V<T: AD> = Vec<T>;
}

impl<T: AD> O3DVec<T> for Vector3<T> {
    type Category = O3DVecCategoryVector3;

    #[inline(always)]
    fn as_any(&self) -> &dyn Any {
        self
    }

    fn type_identifier() -> O3DVecType {
        O3DVecType::NalgebraVector3
    }

    #[inline(always)]
    fn x(&self) -> T {
        self.x
    }

    #[inline(always)]
    fn y(&self) -> T {
        self.y
    }

    #[inline(always)]
    fn z(&self) -> T {
        self.z
    }

    #[inline(always)]
    fn from_slice(slice: &[T]) -> Self {
        Vector3::from_column_slice(slice)
    }

    #[inline(always)]
    fn as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn add(&self, other: &Self) -> Self {
        self + other
    }

    #[inline]
    fn sub(&self, other: &Self) -> Self {
        self - other
    }

    #[inline]
    fn scalar_mul_o3dvec(&self, scalar: T) -> Self {
        scalar.mul_by_nalgebra_matrix_ref(self)
        // scalar * self
        // let mut v = [T::zero(); 3];
        // v.iter_mut().zip(self.iter()).for_each(|(x,y)| *x = scalar * *y);
        // Vector3::from_column_slice(&v)
    }

    #[inline]
    fn norm(&self) -> T {
        self.norm()
    }

    #[inline]
    fn dot(&self, other: &Self) -> T {
        self.dot(other)
    }

    #[inline]
    fn cross(&self, other: &Self) -> Self {
        self.cross(other)
    }

    #[inline]
    fn dis(&self, other: &Self) -> T {
        (self - other).norm()
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct O3DVecCategoryVector3;
impl O3DVecCategoryTrait for O3DVecCategoryVector3 {
    type V<T: AD> = Vector3<T>;
}

impl<T: AD> O3DVec<T> for Point3<T> {
    type Category = O3DVecCategoryPoint3;

    #[inline(always)]
    fn as_any(&self) -> &dyn Any {
        self
    }

    #[inline(always)]
    fn type_identifier() -> O3DVecType { O3DVecType::NalgebraPoint3 }

    fn x(&self) -> T {
        self.x
    }

    fn y(&self) -> T {
        self.y
    }

    fn z(&self) -> T {
        self.z
    }

    fn from_slice(slice: &[T]) -> Self {
        Point3::from_slice(slice)
    }

    fn as_slice(&self) -> &[T] {
        self.coords.as_slice()
    }

    fn add(&self, other: &Self) -> Self {
        (&self.coords + &other.coords).into()
    }

    fn sub(&self, other: &Self) -> Self {
        (&self.coords - &other.coords).into()
    }

    fn scalar_mul_o3dvec(&self, scalar: T) -> Self {
        scalar.mul_by_nalgebra_matrix_ref(&self.coords).into()
    }

    fn norm(&self) -> T {
        self.coords.norm()
    }

    fn dot(&self, other: &Self) -> T {
        self.coords.dot(&other.coords)
    }

    fn cross(&self, other: &Self) -> Self {
        self.coords.cross(&other.coords).into()
    }

    fn dis(&self, other: &Self) -> T {
        (self - other).norm()
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct O3DVecCategoryPoint3;
impl O3DVecCategoryTrait for O3DVecCategoryPoint3 {
    type V<T: AD> = Point3<T>;
}

pub fn o3d_vec_custom_serialize<S, T: AD, V: O3DVec<T>>(value: &V, serializer: S) -> Result<S::Ok, S::Error> where S: serde::Serializer {
    let slice = value.as_slice();
    let slice_as_f64 = [ slice[0].to_constant(), slice[1].to_constant(), slice[2].to_constant() ];
    let mut tuple = serializer.serialize_tuple(3)?;
    for element in &slice_as_f64 {
        tuple.serialize_element(element)?;
    }
    tuple.end()
}

struct O3dVecMyVisitor<T2: AD, V2: O3DVec<T2>> {
    _phantom_data: PhantomData<(T2, V2)>
}

impl<'de, T2: AD, V2: O3DVec<T2>> Visitor<'de> for O3dVecMyVisitor<T2, V2> {
    type Value = V2;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a tuple of size 3")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let x: f64 = seq.next_element().expect("error").expect("error");
        let y: f64 = seq.next_element().expect("error").expect("error");
        let z: f64 = seq.next_element().expect("error").expect("error");
        let xad = T2::constant(x);
        let yad = T2::constant(y);
        let zad = T2::constant(z);
        Ok(V2::from_slice(&[xad, yad, zad]))
    }
}

pub fn o3d_vec_custom_deserialize<'de, D, T: AD, V: O3DVec<T>>(deserializer: D) -> Result<V, D::Error>
where
    D: Deserializer<'de>,
{
    deserializer.deserialize_tuple(3, O3dVecMyVisitor::<T, V> { _phantom_data: PhantomData::default() })
}

pub struct SerdeO3DVec<T: AD, V: O3DVec<T>>(pub V, PhantomData<T>);

impl<T: AD, V: O3DVec<T>> SerializeAs<V> for SerdeO3DVec<T, V> {
    fn serialize_as<S>(source: &V, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
        o3d_vec_custom_serialize(source, serializer)
    }
}
impl<'de, T: AD, V: O3DVec<T>> DeserializeAs<'de, V> for SerdeO3DVec<T, V> {
    fn deserialize_as<D>(deserializer: D) -> Result<V, D::Error> where D: Deserializer<'de> {
        o3d_vec_custom_deserialize(deserializer)
    }
}