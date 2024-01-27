use std::time::Instant;
use bevy_ecs::world::World;

fn main() {
    let start = Instant::now();
    for _ in 0..1000 {
        let world = World::new();
    }
    println!("{:?}", start.elapsed());
}