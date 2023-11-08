use std::any::Any;
use std::fmt::Formatter;
use std::hash::{Hash, Hasher};
use std::marker::PhantomData;
use std::sync::{OnceLock};
use ahash::{AHashMap};
use fnv::FnvHasher;
use serde::de::{DeserializeOwned, SeqAccess, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde::ser::SerializeSeq;
use optima_file::traits::ToJsonString;
use optima_file::traits::FromJsonString;

#[derive(Debug, Clone)]
pub struct AHashMapWrapper<K : Serialize + DeserializeOwned + Hash + Eq, V: Serialize + DeserializeOwned> {
    pub hashmap: AHashMap<K, V>
}
impl<K : Serialize + DeserializeOwned + Hash + Eq, V: Serialize + DeserializeOwned> AHashMapWrapper<K, V> {
    pub fn new() -> Self {
        Self {
            hashmap: AHashMap::new(),
        }
    }
}
impl<K : Serialize + DeserializeOwned + Hash + Eq, V: Serialize + DeserializeOwned> Serialize for AHashMapWrapper<K, V> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error> where S: Serializer {
        let mut seq = serializer.serialize_seq(None).expect("error");

        self.hashmap.iter().for_each(|(k, v)| {
            // seq.serialize_element( &(k.to_json_string(), v.to_json_string()) ).expect("error");
            seq.serialize_element(&(k.to_json_string(), v.to_json_string())).expect("error");
        });

        seq.end()
    }
}
impl<'de, K : Serialize + DeserializeOwned + Hash + Eq, V: Serialize + DeserializeOwned> Deserialize<'de> for AHashMapWrapper<K, V> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error> where D: Deserializer<'de> {
        deserializer.deserialize_seq(AHashMapWrapperVisitor(PhantomData::default()))
    }
}

pub struct AHashMapWrapperVisitor<K : Serialize + DeserializeOwned + Hash + Eq, V: Serialize + DeserializeOwned>(PhantomData<(K, V)>);
impl<'de, K : Serialize + DeserializeOwned + Hash + Eq, V: Serialize + DeserializeOwned> Visitor<'de> for AHashMapWrapperVisitor<K, V> {
    type Value = AHashMapWrapper<K, V>;

    fn expecting(&self, formatter: &mut Formatter) -> std::fmt::Result {
        formatter.write_str("a sequence of serializable and deserializable data")
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error> where A: SeqAccess<'de> {
        let mut out = AHashMapWrapper { hashmap: AHashMap::new() };

        'l: loop {
            let element: Option<(String, String)> = seq.next_element().expect("error");
            match element {
                None => { break 'l; }
                Some(el) => {
                    let k_str = el.0;
                    let v_str = el.1;

                    let k = K::from_json_string(&k_str);
                    let v = V::from_json_string(&v_str);
                    out.hashmap.insert(k, v);
                }
            }
        }

        Ok(out)
    }
}

static mut GLOBAL_UNIVERSAL_HASHMAP: OnceLock<Vec<UniversalHashmap>> = OnceLock::new();

pub struct GUH;
impl GUH {
    #[inline(always)]
    pub fn insert<K : Hash, V: Any>(thread_idx: usize, key: K, value: V) {
        let hashmap = unsafe {
            GLOBAL_UNIVERSAL_HASHMAP.get_or_init(|| {
                let mut v = Vec::new();
                for _ in 0..num_cpus::get() {
                    v.push(UniversalHashmap::new())
                }
                v
            });
            GLOBAL_UNIVERSAL_HASHMAP.get_mut()
        }.unwrap();
        hashmap[thread_idx].insert(key, value);
    }
    #[inline(always)]
    pub fn get_ref<K : Hash, V: Any>(thread_idx: usize, key: &K) -> Option<&V> {
        let hashmap = unsafe {  GLOBAL_UNIVERSAL_HASHMAP.get_or_init(|| vec![UniversalHashmap::new()]); GLOBAL_UNIVERSAL_HASHMAP.get() }.unwrap();
        hashmap[thread_idx].get_ref(key)
    }
    #[inline(always)]
    pub fn mut_ref<K : Hash, V: Any>(thread_idx: usize, key: &K) -> Option<&mut V> {
        let hashmap = unsafe {  GLOBAL_UNIVERSAL_HASHMAP.get_or_init(|| vec![UniversalHashmap::new()]); GLOBAL_UNIVERSAL_HASHMAP.get_mut() }.unwrap();
        hashmap[thread_idx].mut_ref(key)
    }
}

pub struct AnyHashmap<K : Hash + Eq + PartialEq> {
    hashmap: AHashMap<K, Box<dyn Any>>,
}
impl<K : Hash + Eq + PartialEq> AnyHashmap<K> {
    pub fn new() -> Self {
        Self {
            hashmap: AHashMap::new()
        }
    }
    #[inline(always)]
    pub fn insert<V: Any>(&mut self, key: K, value: V) {
        self.hashmap.insert(key, Box::new(value));
    }
    #[inline(always)]
    pub fn get_ref<V: Any>(&self, key: &K) -> Option<&V> {
        let res = self.hashmap.get(key);
        match res {
            None => { None }
            Some(v) => {
                let r = v.as_ref().downcast_ref::<V>();
                match r {
                    None => { None }
                    Some(v) => { Some(v) }
                }
            }
        }
    }
    #[inline(always)]
    pub fn mut_ref<V: Any>(&mut self, key: &K) -> Option<&mut V> {
        let res = self.hashmap.get_mut(key);
        match res {
            None => { None }
            Some(v) => {
                let r = v.as_mut().downcast_mut::<V>();
                match r {
                    None => { None }
                    Some(v) => { Some(v) }
                }
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
pub struct UniversalHashmap {
    hashmap: AnyHashmap<u64>
}
impl UniversalHashmap {
    pub fn new() -> Self {
        Self {
            hashmap: AnyHashmap::new()
        }
    }
    #[inline(always)]
    pub fn insert<K : Hash, V: Any>(&mut self, key: K, value: V) {
        let mut hasher = FnvHasher::default();
        key.hash(&mut hasher);
        let key = hasher.finish();
        self.hashmap.insert(key, value);
    }
    #[inline(always)]
    pub fn get_ref<K : Hash, V: Any>(&self, key: &K) -> Option<&V> {
        let mut hasher = FnvHasher::default();
        key.hash(&mut hasher);
        let key = hasher.finish();
        self.hashmap.get_ref(&key)
    }
    /// Should only be used sparingly, if you need mutability you should store the object as an RwLock or something
    #[inline(always)]
    pub (crate) fn mut_ref<K : Hash, V: Any>(&mut self, key: &K) -> Option<&mut V> {
        let mut hasher = FnvHasher::default();
        key.hash(&mut hasher);
        let key = hasher.finish();
        self.hashmap.mut_ref(&key)
    }
}
unsafe impl Sync for UniversalHashmap { }
unsafe impl Send for UniversalHashmap { }


