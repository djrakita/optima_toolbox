use bevy::prelude::Resource;
use optima_universal_hashmap::{AnyHashmap};

#[derive(Resource)]
pub struct BevyAnyHashmap(pub AnyHashmap<String>);
unsafe impl Send for BevyAnyHashmap { }
unsafe impl Sync for BevyAnyHashmap { }