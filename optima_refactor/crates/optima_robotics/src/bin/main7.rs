use ad_trait::AD;
use ad_trait::forward_ad::adf::adf_f32x2;
use serde::{Deserialize, Serialize};
use optima_file::traits::{FromJsonString, ToJsonString};
use optima_universal_hashmap::AHashMapWrapper;
use serde_with::*;

#[serde_as]
#[derive(Clone, Serialize, Deserialize, Debug)]
struct Test<T: AD> {
    #[serde_as(as = "AHashMapWrapper<(u64, u64), T>")]
    h: AHashMapWrapper<(u64, u64), T>
}

fn main() {
    let mut t = Test { h: AHashMapWrapper::new() };
    t.h.hashmap.insert((1,1), 1.0);
    t.h.hashmap.insert((1,2), 2.0);

    let s = t.to_json_string();
    println!("{:?}", s);

    let t2 = Test::<adf_f32x2>::from_json_string(&s);
    println!("{:?}", t2);
}