use std::collections::HashMap;
use std::time::Instant;
use as_any::{AsAny, Downcast};
use parry_ad::na::Isometry3;
use parry_ad::shape::Ball;
use optima_proximity2::shapes::OParryShape;
use ahash::{AHashMap, HashMapExt};

fn main() {
    let mut v: Vec<Box<dyn AsAny>> = vec![];

    let s = OParryShape::new(Ball::new(1.0), Isometry3::identity());
    v.push(Box::new(s));

    let res = v[0].as_ref().downcast_ref::<OParryShape<f64, Isometry3<f64>>>();

    println!("{:?}", res.is_some());

    /*
    let mut map: AHashMap<(usize, usize, usize), usize> = AHashMap::new();
    for i in 0..10000 {
        for j in 0..1000 {
            map.insert((i,j, 1), 1);
        }
    }

    let start = Instant::now();
    for _ in 0..1000 {
        let res = map.get(&(1,2, 1));
    }
    println!("{:?}", start.elapsed());


    let mut map: HashMap<(usize, usize, usize), usize> = HashMap::new();
    for i in 0..10000 {
        for j in 0..1000 {
            map.insert((i,j, 1), 1);
        }
    }

    let start = Instant::now();
    for _ in 0..1000 {
        let res = map.get(&(1,2,1));
    }
    println!("{:?}", start.elapsed());
    */
}