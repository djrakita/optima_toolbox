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

pub trait ImmutArrTraitRaw<U> {
    fn from_slice(slice: &[U]) -> Self where U: Clone;
    fn as_slice(&self) -> &[U];
    fn contains_element(&self, element: &U) -> bool where U: PartialEq;
    fn len(&self) -> usize;
    fn get_element(&self, i: usize) -> &U;
    /// start idx is inclusive; end idx is exclusive
    #[inline(always)]
    fn subslice(&self, start: usize, end: usize) -> &[U] {
        &self.as_slice()[start..end]
    }
}
pub trait MutArrTraitRaw<U> : ImmutArrTraitRaw<U> {
    fn as_mut_slice(&mut self) -> &mut [U];
    fn get_element_mut(&mut self, i: usize) -> &mut U;
    fn push(&mut self, item: U);
}

pub trait ImmutArrTrait<U: Serialize + DeserializeOwned + Clone + Debug> :
    ImmutArrTraitRaw<U> + Serialize + DeserializeOwned + Default + Clone + Debug + FromIterator<U> + IntoIterator
{ }

pub trait MutArrTrait<U: Serialize + DeserializeOwned + Clone + Debug > : ImmutArrTrait<U>
{ }

impl<U: Serialize + DeserializeOwned + Clone + Debug, A> ImmutArrTrait<U> for A
where A: ImmutArrTraitRaw<U> + Serialize + DeserializeOwned + Default + Clone + Debug + FromIterator<U> + IntoIterator
{ }

impl<U: Serialize + DeserializeOwned + Clone + Debug, A> MutArrTrait<U> for A
where A: MutArrTraitRaw<U> + Serialize + DeserializeOwned + Default + Clone + Debug + FromIterator<U> + IntoIterator
{ }

impl<U> ImmutArrTraitRaw<U> for Vec<U> {
    #[inline]
    fn from_slice(slice: &[U]) -> Self where U: Clone {
        Vec::from(slice)
    }

    #[inline]
    fn as_slice(&self) -> &[U] {
        self.as_slice()
    }

    #[inline]
    fn contains_element(&self, element: &U) -> bool where U: PartialEq {
        self.contains(element)
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }

    #[inline]
    fn get_element(&self, i: usize) -> &U {
        &self[i]
    }
}
impl<U> MutArrTraitRaw<U> for Vec<U> {
    fn as_mut_slice(&mut self) -> &mut [U] {
        self.as_mut_slice()
    }

    fn get_element_mut(&mut self, i: usize) -> &mut U {
        &mut self[i]
    }

    fn push(&mut self, item: U) {
        self.push(item);
    }
}

impl<'a, U> ImmutArrTraitRaw<U> for &'a [U] {
    fn from_slice(_slice: &[U]) -> Self where U: Clone {
        unimplemented!("not possible to return slice from slice")
    }

    fn as_slice(&self) -> &[U] {
        self
    }

    fn contains_element(&self, element: &U) -> bool where U: PartialEq {
        self.contains(element)
    }

    fn len(&self) -> usize {
        <[U]>::len(self)
    }

    fn get_element(&self, i: usize) -> &U {
        self.get(i).expect("error")
    }
}
impl<'a, U> ImmutArrTraitRaw<U> for &'a mut [U] {
    fn from_slice(_slice: &[U]) -> Self where U: Clone {
        unimplemented!("not possible to return slice from slice")
    }

    fn as_slice(&self) -> &[U] {
        self
    }

    fn contains_element(&self, element: &U) -> bool where U: PartialEq {
        self.contains(element)
    }

    fn len(&self) -> usize {
        <[U]>::len(self)
    }

    fn get_element(&self, i: usize) -> &U {
        self.get(i).expect("error")
    }
}
impl<'a, U> MutArrTraitRaw<U> for &'a mut [U] {
    fn as_mut_slice(&mut self) -> &mut [U] {
        self
    }

    fn get_element_mut(&mut self, i: usize) -> &mut U {
        self.get_mut(i).expect("error")
    }

    fn push(&mut self, _item: U) {
        unimplemented!("cannot push into a mut slice.")
    }
}

impl<U, const M: usize> ImmutArrTraitRaw<U> for ArrayVec<U, M> {
    #[inline]
    fn from_slice(slice: &[U]) -> Self where U: Clone {
        ArrayVec::try_from(slice).expect("error")
    }

    #[inline]
    fn as_slice(&self) -> &[U] {
        self.as_slice()
    }

    #[inline]
    fn contains_element(&self, element: &U) -> bool where U: PartialEq {
        self.contains(element)
    }

    #[inline]
    fn len(&self) -> usize {
        self.len()
    }

    #[inline]
    fn get_element(&self, i: usize) -> &U {
        &self[i]
    }
}
impl<U, const M: usize> MutArrTraitRaw<U> for ArrayVec<U, M> {
    fn as_mut_slice(&mut self) -> &mut [U] {
        self.as_mut_slice()
    }

    fn get_element_mut(&mut self, i: usize) -> &mut U {
        &mut self[i]
    }

    fn push(&mut self, item: U) {
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


