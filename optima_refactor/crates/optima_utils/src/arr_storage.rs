use std::fmt::Debug;
use std::hash::Hash;
use std::str::{Chars, Split};
use std::str::pattern::Pattern;
use arrayvec::{ArrayString, ArrayVec};
use serde::{Serialize, Deserialize};
use serde::de::DeserializeOwned;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VecStor;
impl ArrStorageTrait for VecStor {
    type ArrType<U: Serialize + DeserializeOwned + Clone + Debug> = Vec<U>;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ArrayVecStor<const M: usize>;
impl<const M: usize> ArrStorageTrait for ArrayVecStor<M> {
    type ArrType<U: Serialize + DeserializeOwned + Clone + Debug> = ArrayVec<U, M>;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StringStor;
impl StrStorageTrait for StringStor {
    type StrType = String;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ArrayStringStor<const M: usize>;
impl<const M: usize> StrStorageTrait for ArrayStringStor<M> {
    type StrType = ArrayString<M>;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ArrStorageTrait: Clone + Debug + Serialize + DeserializeOwned {
    type ArrType<U: Serialize + DeserializeOwned + Clone + Debug> : MutArrTrait<U>;
}

pub trait StrStorageTrait: Clone + Debug + Serialize + DeserializeOwned {
    type StrType : StrTrait;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ImmutArrTrait<T: Serialize + DeserializeOwned + Clone + Debug> :
    Serialize + DeserializeOwned + Default + Clone + Debug + FromIterator<T> + IntoIterator
{
    fn from_slice(slice: &[T]) -> Self;
    fn as_slice(&self) -> &[T];
    fn contains_element(&self, element: &T) -> bool where T: PartialEq;
    fn len(&self) -> usize;
    fn get_element(&self, i: usize) -> &T;
}

pub trait MutArrTrait<T: Serialize + DeserializeOwned + Clone + Debug > :
    ImmutArrTrait<T>
{
    fn as_mut_slice(&mut self) -> &mut [T];
    fn get_element_mut(&mut self, i: usize) -> &mut T;
    fn push(&mut self, item: T);
}

impl<T: Serialize + DeserializeOwned + Clone + Debug> ImmutArrTrait<T> for Vec<T> {
    #[inline]
    fn from_slice(slice: &[T]) -> Self {
        Vec::from(slice)
    }

    #[inline]
    fn as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn contains_element(&self, element: &T) -> bool where T: PartialEq {
        self.contains(element)
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }

    #[inline]
    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }
}
impl<T: Serialize + DeserializeOwned + Clone + Debug> MutArrTrait<T> for Vec<T> {
    fn as_mut_slice(&mut self) -> &mut [T] {
        self.as_mut_slice()
    }

    fn get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    fn push(&mut self, item: T) {
        self.push(item);
    }
}

impl<T: Serialize + DeserializeOwned + Clone + Debug, const M: usize> ImmutArrTrait<T> for ArrayVec<T, M> {
    #[inline]
    fn from_slice(slice: &[T]) -> Self {
        ArrayVec::try_from(slice).expect("error")
    }

    #[inline]
    fn as_slice(&self) -> &[T] {
        self.as_slice()
    }

    #[inline]
    fn contains_element(&self, element: &T) -> bool where T: PartialEq {
        self.contains(element)
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }

    #[inline]
    fn get_element(&self, i: usize) -> &T {
        &self[i]
    }
}
impl<T: Serialize + DeserializeOwned + Clone + Debug, const M: usize> MutArrTrait<T> for ArrayVec<T, M> {
    fn as_mut_slice(&mut self) -> &mut [T] {
        self.as_mut_slice()
    }

    fn get_element_mut(&mut self, i: usize) -> &mut T {
        &mut self[i]
    }

    fn push(&mut self, item: T) {
        self.push(item);
    }
}

pub trait StrTrait: Serialize + DeserializeOwned + Clone + Debug + Default + PartialEq + Eq + Hash {
    fn from_str(s: &str) -> Self;
    fn as_str(&self) -> &str;
    fn push(&mut self, c: char);
    fn push_str(&mut self, s: &str);
    fn characters(&self) -> Chars<'_>;
    fn split_at_pattern<'a, P: Pattern<'a>>(&'a self, pat: P) -> Split<'a, P>;
    fn len(&self) -> usize;
}

impl StrTrait for String {
    fn from_str(s: &str) -> Self {
        String::from(s)
    }

    fn as_str(&self) -> &str {
        self.as_str()
    }

    fn push(&mut self, c: char) {
        self.push(c)
    }

    fn push_str(&mut self, s: &str) {
        self.push_str(s)
    }

    fn characters(&self) -> Chars<'_> {
        self.chars()
    }

    fn split_at_pattern<'a, P: Pattern<'a>>(&'a self, pat: P) -> Split<'a, P> {
        self.split(pat)
    }

    fn len(&self) -> usize {
        self.len()
    }
}
impl<const M: usize> StrTrait for ArrayString<M> {
    fn from_str(s: &str) -> Self {
        Self::from(s).expect("error")
    }

    fn as_str(&self) -> &str {
        self.as_str()
    }

    fn push(&mut self, c: char) {
        self.push(c)
    }

    fn push_str(&mut self, s: &str) {
        self.push_str(s)
    }

    fn characters(&self) -> Chars<'_> {
        self.chars()
    }

    fn split_at_pattern<'a, P: Pattern<'a>>(&'a self, pat: P) -> Split<'a, P> {
        self.split(pat)
    }

    fn len(&self) -> usize {
        self.len()
    }
}


