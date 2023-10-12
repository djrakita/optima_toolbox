
pub struct Test {
    a: Vec<usize>,
    b: Vec<usize>,
    c: usize
}
impl Test {
    fn refs(&self) -> Vec<&usize> {
        let mut out = vec![];
        self.a.iter().for_each(|x| { out.push(x) });
        self.b.iter().for_each(|x| { out.push(x) });
        out.push(&self.c);
        out
    }
}

#[derive(Debug)]
pub struct Test2<'a> {
    refs: Vec<&'a usize>
}

fn main() {
    let t = Test { a: vec![1,2,3,4,5,6,7,8,9], b: vec![10,11,12], c: 13 };
    let r = t.refs();
    let t2 = Test2 { refs: r };

    println!("{:?}", t2);
}