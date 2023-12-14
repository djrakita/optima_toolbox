
fn main() {
    let mut v = vec![1,6,3,9];
    v.sort_by(|x, y| y.cmp(&x));
    println!("{:?}", v);
}