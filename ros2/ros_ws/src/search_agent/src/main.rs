use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
mod ros2;

use clap::{self, Parser, ValueEnum};
use r2r::{self, geometry_msgs, log_error, log_warn, log_info, QosProfile};
use robcore::RobotId;
use ros2::{
    agent_msg_to_ros2_msg, cov_pose_to_pose2d, ros2_msg_to_agent_msg, scan_to_lidar_data, Ros2,
};

fn behaviors() -> Vec<String> {
    robcore::behaviors::Behavior::value_variants()
        .iter()
        .filter_map(|v| Some(v.to_possible_value()?.get_name().to_string()))
        .collect()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut agent = Ros2::new();

    let logger = agent.node.logger().to_string().leak();

    let Ok(behavior_param) = agent.node.get_parameter::<String>("behavior") else {
        return Err(format!(
            "No `behavior` parameter provided. Please provide one. Possible values: [{}]",
            behaviors().join(", ")
        )
        .into());
    };

    let Ok(behavior) =
        robcore::behaviors::Behavior::from_str(&behavior_param, true).map(|b| b.get_fn())
    else {
        return Err(format!(
            "Invalid `behavior` parameter provided. Possible values: [{}]",
            behaviors().join(", ")
        )
        .into());
    };

    let publisher = agent
        .node
        .create_publisher::<geometry_msgs::msg::Twist>("cmd_vel", QosProfile::default())
        .unwrap();

    let mut robot = robcore::Robot::default();
    let id = match agent.node.get_parameter::<i64>("id") {
        Ok(id) => id.try_into().ok(),
        Err(_) => (|| {
            // Try to parse id from node namespace
            let namespace = agent.node.namespace().ok()?;
            let (_, end) = namespace.split_once("/robot_")?;
            end.parse().ok()
        })(),
    }
    .unwrap_or_default();
    robot.id = RobotId::new(id);

    log_info!(logger, "Robot ID: {}", id);

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
                let msgs = guard.drain(..).filter_map(|ros_msg| {
                    let sender_id = ros_msg.sender_id;
                    match ros2_msg_to_agent_msg(ros_msg) {
                        Some(msg) => Some(msg),
                        None => {
                            log_warn!(logger, "Error converting message from: {}", sender_id);
                            None
                        }
                    }
                });
                robot.incoming_msg.extend(msgs);
            }

            // Publish outgoing msgs
            {
                robot.outgoing_msg.drain(..).for_each(|msg| {
                    if let Some(ros_msg) = agent_msg_to_ros2_msg(msg) {
                        agent.outgoing_msgs_pub.publish(&ros_msg).unwrap();
                    } else {
                        log_warn!(logger, "Error converting message to ROS2");
                    }
                });
            }
        }

        let control = behavior(&mut robot, Instant::now());

        // Only linear x and angular z are used by robot
        let mut twist = geometry_msgs::msg::Twist::default();
        twist.linear.x = control.speed as f64;
        twist.angular.z = control.steer as f64;

        if let Err(e) = publisher.publish(&twist) {
            log_warn!(logger, "Error publishing twist: {}", e);
        }
    }
}
