use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
mod ros2;

use clap::{self, Parser};
use r2r::{self, geometry_msgs, QosProfile};
use ros2::{agent_msg_to_ros2_msg, cov_pose_to_pose2d, ros2_msg_to_agent_msg, scan_to_lidar_data, Ros2};

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

        // Set inputs for robot
        {
            // Set the robot lidar input
            if let Some(scan) = agent.scan.lock().unwrap().take() {
                let lidar = scan_to_lidar_data(&scan);
                robot.lidar = lidar;
            }

            // Set the robot pose
            if let Some(pose) = agent.pose.lock().unwrap().take() {
                let (pos, angle) = cov_pose_to_pose2d(&pose);
                robot.pos = pos;
                robot.angle = angle;
            }

            // Set incomming messages
            {
                let mut guard = agent.incomming_msgs.lock().unwrap();
                let msgs = guard.drain(..).map(|msg| ros2_msg_to_agent_msg(&msg));
                robot.incoming_msg.extend(msgs);
            }

            // Publish outgoing msgs
            {
                robot.outgoing_msg.drain(..).for_each(|msg| {
                    let ros_msg = agent_msg_to_ros2_msg(&msg);
                    agent.outgoing_msgs_pub.publish(&ros_msg).unwrap();
                });
            }
        }

        let control = behavior(&mut robot, Instant::now());

        let mut twist = geometry_msgs::msg::Twist::default();

        let m = r2r::search_agent_msgs::msg::AgentMessage::default();

        // We can control
        twist.linear.x = control.speed as f64;
        twist.angular.z = control.steer as f64;
        publisher.publish(&twist).unwrap();
    }
}
