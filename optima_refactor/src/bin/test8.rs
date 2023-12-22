

fn main() {
    let a = vec![1,5,8,2,5,3,7,9,0,6,4,1];
    let mut indices = (0..a.len()).collect::<Vec<_>>();
    indices.sort_unstable_by_key(|i| &a[*i]);
    println!("{:?}", indices);
}