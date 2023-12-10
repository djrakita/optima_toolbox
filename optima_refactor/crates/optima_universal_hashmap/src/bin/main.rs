use ahash::AHashMap;

fn main() {
    let mut a = AHashMap::new();
    a.insert(0, vec![0,1,2]);
    a.insert(1, vec![1,2,3]);
    a.insert(2, vec![4,5,0]);

    a.iter().for_each(|x| {
       println!("{:?}", x);
    });

    // a = a.iter().filter(|x| x.1.contains(&0)).map(|(x,y)| (*x, y.clone()) ).collect();

    a.iter_mut().for_each(|x| {
        let idx = x.1.iter().position(|x| *x == 0);
        match idx {
            None => {  }
            Some(idx) => { x.1.remove(idx); }
        }
    });

    a.iter_mut().for_each(|x| {
        let idx = x.1.iter().position(|x| *x == 1);
        match idx {
            None => {  }
            Some(idx) => { x.1.remove(idx); }
        }
    });

    a.iter_mut().for_each(|x| {
        let idx = x.1.iter().position(|x| *x == 2);
        match idx {
            None => {  }
            Some(idx) => { x.1.remove(idx); }
        }
    });

    a = a.iter().filter(|x| !x.1.is_empty() ).map(|(x,y)|(*x, y.clone())).collect();

    println!("---");

    a.iter().for_each(|x| {
        println!("{:?}", x);
    });
}