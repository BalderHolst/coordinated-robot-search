use std::time::Duration;

use futures::{executor::LocalPool, task::LocalSpawnExt};
use r2r::{
    geometry_msgs::msg::{Twist, Vector3},
    QosProfile,
};
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "multi_robot_controller", "")?;
    let qos = QosProfile::sensor_data();
    let args = std::env::args().collect::<Vec<_>>();
    if args.len() != 2 || args[1] == "-h" || args[1] == "--help" {
        println!("Usage: multi_robot_control <N> ");
        println!("N is the number of robots to control");
        return Ok(());
    }
    let n_robots = args[1].parse::<usize>().expect("Expected number as input!");
    if n_robots < 1 {
        return Err(Box::new(std::io::Error::new(
            std::io::ErrorKind::Other,
            "N must the positive number",
        )));
    }

    let cmd_vel_publishers: Vec<_> = (0..n_robots)
        .map(|i| format!("robot_{i}/cmd_vel"))
        .map(|topic| {
            node.create_publisher::<Twist>(&topic, qos.clone())
                .expect("Could not create publisher")
        })
        .collect();

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();
    // task that every other time forwards message to topic2
    spawner.spawn_local(async move {
        loop {
            for (i, publiser) in cmd_vel_publishers.iter().enumerate() {
                // Code to publish to a topic
                let send = Twist {
                    linear: Vector3 {
                        x: i as f64,
                        y: 0.0,
                        z: 0.0,
                    },
                    ..Default::default()
                };
                publiser.publish(&send).expect("Could not publish");
            }
            // Sleep for a second
            std::thread::sleep(Duration::from_secs(1));
        }
    })?;

    loop {
        node.spin_once(Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
