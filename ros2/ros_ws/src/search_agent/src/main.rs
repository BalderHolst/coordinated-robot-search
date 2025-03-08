use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};
mod convert_msg;

use clap::{self, Parser, ValueEnum};
use convert_msg::{
    agent_msg_to_ros2_msg, cov_pose_to_pose2d, ros2_msg_to_agent_msg, scan_to_lidar_data,
};
use futures::{executor::LocalPool, task::LocalSpawnExt, StreamExt};
use r2r::{
    self, geometry_msgs, log_error, log_info, log_warn, search_agent_msgs, sensor_msgs, Publisher,
    QosProfile,
};
use botbrain::{behaviors::BehaviorFn, RobotId};

const DEFAULT_CHANNEL_TOPIC: &str = "/search_channel";

pub struct SearchAgent {
    node: r2r::Node,
    robot: botbrain::Robot,
    nl: String,
    behavior: BehaviorFn,
    pool: LocalPool,
    pose: Arc<Mutex<Option<geometry_msgs::msg::PoseWithCovarianceStamped>>>,
    scan: Arc<Mutex<Option<sensor_msgs::msg::LaserScan>>>,
    incomming_msgs: Arc<Mutex<Vec<search_agent_msgs::msg::AgentMessage>>>,
    outgoing_msgs_pub: Publisher<search_agent_msgs::msg::AgentMessage>,
    cmd_pub: Publisher<geometry_msgs::msg::Twist>,
    // pub map: Arc<Mutex<Option<r2r::nav_msgs::msg::OccupancyGrid>>>,
    // had_map_update: Arc<Mutex<bool>>,
}

impl SearchAgent {
    fn behaviors() -> Vec<String> {
        botbrain::behaviors::Behavior::value_variants()
            .iter()
            .filter_map(|v| Some(v.to_possible_value()?.get_name().to_string()))
            .collect()
    }

    pub fn new() -> Self {
        let ctx = r2r::Context::create().unwrap();

        let mut node = r2r::Node::create(ctx, "agent", "").unwrap();

        let nl = node.logger().to_string();

        let Ok(behavior_param) = node.get_parameter::<String>("behavior") else {
            log_error!(
                &nl,
                "No `behavior` parameter provided. Please provide one. Possible values: [{}]",
                Self::behaviors().join(", ")
            );
            std::process::exit(1);
        };

        let Ok(behavior) =
            botbrain::behaviors::Behavior::from_str(&behavior_param, true).map(|b| b.get_fn())
        else {
            log_error!(
                &nl,
                "Invalid `behavior` parameter provided. Possible values: [{}]",
                Self::behaviors().join(", ")
            );
            std::process::exit(1);
        };

        let mut robot = botbrain::Robot::default();
        let id = match node.get_parameter::<i64>("id") {
            Ok(id) => id.try_into().ok(),
            Err(_) => (|| {
                // Try to parse id from node namespace
                let namespace = node.namespace().ok()?;
                let (_, end) = namespace.split_once("/robot_")?;
                end.parse().ok()
            })(),
        }
        .unwrap_or_default();

        log_info!(&nl, "Robot ID: {}", id);
        robot.id = RobotId::new(id);

        let channel_topic: String = node
            .get_parameter("channel_topic")
            .unwrap_or(DEFAULT_CHANNEL_TOPIC.into());

        let qos = QosProfile::default().volatile();
        let mut sub_scan = node
            .subscribe::<sensor_msgs::msg::LaserScan>("scan", qos)
            .unwrap();

        let qos = QosProfile::default().transient_local();
        let mut sub_pose = node
            .subscribe::<r2r::geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", qos)
            .unwrap();

        let qos = QosProfile::default().volatile();
        let msgs_pub = node
            .create_publisher::<search_agent_msgs::msg::AgentMessage>(&channel_topic, qos.clone())
            .unwrap();
        let mut msgs_sub = node
            .subscribe::<search_agent_msgs::msg::AgentMessage>(&channel_topic, qos)
            .unwrap();

        let qos = QosProfile::default().volatile();
        let cmd_pub = node
            .create_publisher::<geometry_msgs::msg::Twist>("cmd_vel", qos)
            .unwrap();

        let scan = Arc::new(Mutex::new(None));
        let pose = Arc::new(Mutex::new(None));
        let incomming_msgs = Arc::new(Mutex::new(Vec::new()));

        let pool = LocalPool::new();
        {
            let nl = nl.clone();
            let scan = scan.clone();
            pool.spawner()
                .spawn_local(async move {
                    log_info!(&nl, "Scan listener started");
                    while let Some(msg) = sub_scan.next().await {
                        scan.lock().unwrap().replace(msg);
                    }
                    log_info!(&nl, "Scan listener broken");
                })
                .unwrap();
        }
        {
            let nl = nl.clone();
            let pose = pose.clone();
            pool.spawner()
                .spawn_local(async move {
                    log_info!(&nl, "Pose listener started");
                    while let Some(msg) = sub_pose.next().await {
                        pose.lock().unwrap().replace(msg);
                    }
                })
                .unwrap();
        }
        {
            let nl = nl.clone();
            let incomming_msgs = incomming_msgs.clone();
            pool.spawner()
                .spawn_local(async move {
                    log_info!(&nl, "Message listener started on '{}'", channel_topic);
                    while let Some(msg) = msgs_sub.next().await {
                        incomming_msgs.lock().unwrap().push(msg);
                    }
                    log_info!(&nl, "Message listener broken");
                })
                .unwrap();
        }
        Self {
            node,
            robot,
            behavior,
            nl,
            pool,
            scan,
            pose,
            incomming_msgs,
            outgoing_msgs_pub: msgs_pub,
            cmd_pub,
        }
    }

    pub fn run(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        loop {
            self.node.spin_once(Duration::from_secs(1));
            self.pool.run_until_stalled();

            // Set inputs for robot
            {
                // Set the robot lidar input
                if let Some(scan) = self.scan.lock().unwrap().take() {
                    let lidar = scan_to_lidar_data(&scan);
                    self.robot.lidar = lidar;
                }

                // Set the robot pose
                if let Some(pose) = self.pose.lock().unwrap().take() {
                    let (pos, angle) = cov_pose_to_pose2d(&pose);
                    self.robot.pos = pos;
                    self.robot.angle = angle;
                }

                // Set incomming messages
                {
                    let mut guard = self.incomming_msgs.lock().unwrap();
                    let msgs = guard.drain(..).filter_map(|ros_msg| {
                        let sender_id = ros_msg.sender_id;
                        match ros2_msg_to_agent_msg(ros_msg) {
                            Some(msg) => Some(msg),
                            None => {
                                log_warn!(&self.nl, "Error converting message from: {}", sender_id);
                                None
                            }
                        }
                    });
                    self.robot.incoming_msg.extend(msgs);
                }

                // Publish outgoing msgs
                {
                    self.robot.outgoing_msg.drain(..).for_each(|msg| {
                        if let Some(ros_msg) = agent_msg_to_ros2_msg(msg) {
                            self.outgoing_msgs_pub.publish(&ros_msg).unwrap();
                        } else {
                            log_warn!(&self.nl, "Error converting message to ROS2");
                        }
                    });
                }
            }

            let control = (self.behavior)(&mut self.robot, Instant::now());

            // Only linear x and angular z are used by robot
            let mut twist = geometry_msgs::msg::Twist::default();
            twist.linear.x = control.speed as f64;
            twist.angular.z = control.steer as f64;

            if let Err(e) = self.cmd_pub.publish(&twist) {
                log_warn!(&self.nl, "Error publishing twist: {}", e);
            }
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut agent = SearchAgent::new();
    agent.run()
}
