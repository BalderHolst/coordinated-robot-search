use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
mod ros2;

use r2r::{self, geometry_msgs, QosProfile};
use ros2::{cov_pose_to_pose2d, scan_to_lidar_data, Ros2};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut agent = Ros2::new();
    let publisher = agent
        .node
        .create_publisher::<geometry_msgs::msg::Twist>("cmd_vel", QosProfile::default())
        .unwrap();
    let mut robot = robcore::Robot::default();
    let behavior = robcore::behaviors::avoid_obstacles;
    loop {
        agent.node.spin_once(Duration::from_secs(1));
        agent.pool.run_until_stalled();

        if let Some(lidar) = agent.scan.lock().unwrap().take() {
            let lidar = scan_to_lidar_data(&lidar);
            // let min = lidar
            //     .0
            //     .iter()
            //     .min_by(|x, y| x.distance.total_cmp(&y.distance));
            robot.lidar = lidar;
        }
        if let Some(pose) = agent.pose.lock().unwrap().take() {
            let (pos, angle) = cov_pose_to_pose2d(&pose);
            // println!("Pose: {:?}", (pos, angle));
            robot.pos = pos;
            robot.angle = angle;
        }
        let control = behavior(&mut robot, Instant::now());
        let mut twist = geometry_msgs::msg::Twist::default();
        // We can control
        twist.linear.x = control.speed as f64;
        twist.angular.z = control.steer as f64;
        publisher.publish(&twist).unwrap();
    }
}
