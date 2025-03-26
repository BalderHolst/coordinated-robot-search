use std::path::PathBuf;

use botbrain::behaviors::Behavior;
use simple_sim::{cli::default_threads, sim::{SimArgs, Simulator}, world::world_from_path};

fn main() {
    let world_path = PathBuf::from("../simple_sim/worlds/objectmap/medium_empty.ron");
    let world = world_from_path(&world_path).unwrap();
    let behavior = Behavior::parse("rl").unwrap();
    let threads = default_threads();
    let sim = Simulator::new(SimArgs {
        world,
        behavior,
        threads, 
    });
}
