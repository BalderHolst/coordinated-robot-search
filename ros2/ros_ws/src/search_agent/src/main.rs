use std::{
    sync::{Arc, Mutex},
    time::Duration,
};
mod ros2;

use r2r;
use ros2::{scan_to_lidar_data, Ros2};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut agent = Ros2::new();
    loop {
        agent.node.spin_once(Duration::from_secs(1));
        agent.pool.run_until_stalled();

        if let Some(lidar) = agent.scan.lock().unwrap().take() {
            let lidar = scan_to_lidar_data(&lidar);
            let min = lidar
                .0
                .iter()
                .min_by(|x, y| x.distance.total_cmp(&y.distance));
            println!("Lidar data: {:?}", min);
        }
    }
}
