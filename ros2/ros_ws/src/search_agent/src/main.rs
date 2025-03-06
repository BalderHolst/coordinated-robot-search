use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
mod ros2;

use r2r::{self, geometry_msgs, QosProfile};
use ros2::{cov_pose_to_pose2d, scan_to_lidar_data, Ros2};
use clap::{self, Parser};

#[derive(Parser)]
struct Cli {
    #[arg(short, long)]
    namespace: Option<String>,

    #[arg(short, long, default_value = "avoid-obstacles")]
    behavior: robcore::behaviors::Behavior,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {

    let args = Cli::parse();

    let mut agent = Ros2::new(args.namespace);

    let publisher = agent
        .node
        .create_publisher::<geometry_msgs::msg::Twist>("cmd_vel", QosProfile::default())
        .unwrap();
    let mut robot = robcore::Robot::default();
    let behavior = args.behavior.get_fn();
    loop {
        agent.node.spin_once(Duration::from_secs(1));
        agent.pool.run_until_stalled();

        if let Some(scan) = agent.scan.lock().unwrap().take() {
            let lidar = scan_to_lidar_data(&scan);
           robot.lidar = lidar;
        }
        if let Some(pose) = agent.pose.lock().unwrap().take() {
            let (pos, angle) = cov_pose_to_pose2d(&pose);
            robot.pos = pos;
            robot.angle = angle;
        }
        let control = behavior(&mut robot, Instant::now());

        let closest_point = robot.lidar.points().min_by(|a, b| {
            a.distance
                .partial_cmp(&b.distance)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        let mut twist = geometry_msgs::msg::Twist::default();

        // We can control
        twist.linear.x = control.speed as f64;
        twist.angular.z = control.steer as f64;
        publisher.publish(&twist).unwrap();
    }
}
