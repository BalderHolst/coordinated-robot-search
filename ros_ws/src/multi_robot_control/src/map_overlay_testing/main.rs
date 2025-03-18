use std::{
    borrow::Borrow,
    sync::{Arc, Mutex},
    time::Duration,
};
mod convert_msg;

use convert_msg::{
    agent_msg_to_ros2_msg, cov_pose_to_pose2d, ros2_msg_to_agent_msg, scan_to_lidar_data,
};
use futures::{StreamExt, executor::LocalPool, task::LocalSpawnExt};
use r2r::{
    self, Publisher, QosProfile, geometry_msgs, log_error, log_info, log_warn, nav_msgs,
    ros_agent_msgs, sensor_msgs,
};
const DEFAULT_CHANNEL_TOPIC: &str = "/search_channel";

pub struct RosAgent {
    node: r2r::Node,
    robot: Box<dyn botbrain::Robot>,
    nl: String,
    behavior: botbrain::behaviors::Behavior,
    pool: LocalPool,
    pose: Arc<Mutex<Option<geometry_msgs::msg::PoseWithCovarianceStamped>>>,
    scan: Arc<Mutex<Option<sensor_msgs::msg::LaserScan>>>,
    incomming_msgs: Arc<Mutex<Vec<ros_agent_msgs::msg::AgentMessage>>>,
    outgoing_msgs_pub: Publisher<ros_agent_msgs::msg::AgentMessage>,
    cmd_pub: Publisher<geometry_msgs::msg::Twist>,
    map: Arc<Mutex<Option<nav_msgs::msg::OccupancyGrid>>>,
    map_overlay_pub: Publisher<nav_msgs::msg::OccupancyGrid>,
}

impl RosAgent {
    pub fn new() -> Self {
        let ctx = r2r::Context::create().unwrap();

        let mut node = r2r::Node::create(ctx, "agent", "").unwrap();

        let nl = node.logger().to_string();

        if node.get_parameter::<bool>("use_sim_time").unwrap_or(false) {
            node.get_time_source()
                .enable_sim_time(&mut node)
                .expect("Could not use sim time");
            log_info!(&nl, "Using simulated time");
        } else {
            log_info!(&nl, "Using system time");
        }

        let Ok(behavior_param) = node.get_parameter::<String>("behavior") else {
            log_error!(
                &nl,
                "No `behavior` parameter provided. Please provide one. Possible values: [{}]",
                botbrain::behaviors::Behavior::behavior_names().join(", ")
            );
            std::process::exit(1);
        };

        let behavior = match botbrain::behaviors::Behavior::parse(&behavior_param) {
            Ok(behavior) => behavior,
            Err(e) => {
                log_error!(&nl, "Invalid `behavior` parameter provided: {}", e);
                std::process::exit(1);
            }
        };

        let mut robot = behavior.create_robot();
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
        robot.set_id(botbrain::RobotId::new(id));

        let pool = LocalPool::new();

        // Subscribe/publish to the channel topic
        let channel_topic: String = node
            .get_parameter("channel_topic")
            .unwrap_or(DEFAULT_CHANNEL_TOPIC.into());
        let incomming_msgs = Arc::new(Mutex::new(Vec::new()));
        let msgs_pub = node
            .create_publisher::<ros_agent_msgs::msg::AgentMessage>(
                &channel_topic,
                QosProfile::default().volatile(),
            )
            .unwrap();
        let mut msgs_sub = node
            .subscribe::<ros_agent_msgs::msg::AgentMessage>(
                &channel_topic,
                QosProfile::default().volatile(),
            )
            .unwrap();
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

        // Subscribe to laser scan
        let mut sub_scan = node
            .subscribe::<sensor_msgs::msg::LaserScan>("scan", QosProfile::sensor_data())
            .unwrap();
        let scan = Arc::new(Mutex::new(None));
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

        // Subscribe to amcl_pose
        let pose = Arc::new(Mutex::new(None));
        let mut sub_pose = node
            .subscribe::<r2r::geometry_msgs::msg::PoseWithCovarianceStamped>(
                "amcl_pose",
                QosProfile::default().transient_local(),
            )
            .unwrap();
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

        // Publish to cmd_vel
        let cmd_pub = node
            .create_publisher::<geometry_msgs::msg::Twist>(
                "cmd_vel",
                QosProfile::default().volatile(),
            )
            .unwrap();

        // Subscribe to map
        let map = Arc::new(Mutex::new(None));
        let mut sub_map = node
            .subscribe::<nav_msgs::msg::OccupancyGrid>(
                "/map",
                QosProfile::default().transient_local(),
            )
            .unwrap();
        {
            let nl = nl.clone();
            let map = map.clone();
            pool.spawner()
                .spawn_local(async move {
                    log_info!(&nl, "Map listener started");
                    while let Some(msg) = sub_map.next().await {
                        map.lock().unwrap().replace(msg);
                        log_info!(&nl, "Map received");
                    }
                })
                .unwrap();
        }

        // Publish to map_overlay
        let map_overlay_pub = node
            .create_publisher::<nav_msgs::msg::OccupancyGrid>(
                "map_overlay",
                QosProfile::default().volatile(),
            )
            .unwrap();

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
            map,
            map_overlay_pub,
        }
    }

    pub fn run(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let mut last_map_update = Duration::default();
        let mut map_val = 0;
        loop {
            self.node.spin_once(Duration::from_secs(1));
            self.pool.run_until_stalled();

            // Set inputs for robot
            {
                // Set the robot lidar input
                if let Some(scan) = self.scan.lock().unwrap().take() {
                    let lidar = scan_to_lidar_data(&scan);
                    self.robot.input_lidar(lidar);
                }

                // Set the robot pose
                if let Some(pose) = self.pose.lock().unwrap().take() {
                    let (pos, angle) = cov_pose_to_pose2d(&pose);
                    self.robot.input_pose(botbrain::RobotPose { pos, angle });
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

                    let postbox = self.robot.get_postbox_mut();

                    // Deposit incoming msgs in the robot postbox
                    postbox.deposit(msgs);

                    // Empty the outgoing messages from the robot postbox and send them to via ROS2
                    postbox.empty().into_iter().for_each(|msg| {
                        if let Some(ros_msg) = agent_msg_to_ros2_msg(msg) {
                            self.outgoing_msgs_pub.publish(&ros_msg).unwrap();
                        } else {
                            log_warn!(&self.nl, "Error converting message to ROS2");
                        }
                    });
                }
            }

            let time = self.node.get_ros_clock().lock().unwrap().get_now()?;
            if time.is_zero() {
                // Simulated time is not available yet
                log_warn!(&self.nl, "Time not ready yet: {:?}", time);
                std::thread::sleep(std::time::Duration::from_secs(1));
                continue;
            }

            let control = (self.behavior.behavior_fn())(&mut self.robot, time);

            // Only linear x and angular z are used by robot
            let mut twist = geometry_msgs::msg::Twist::default();
            twist.linear.x = control.speed as f64;
            twist.angular.z = control.steer as f64;

            if time > last_map_update + Duration::from_secs_f32(0.5) {
                last_map_update = time;
                if let Some(mut map) = self.map.lock().unwrap().clone() {
                    map.data.iter_mut().for_each(|v| *v = map_val);
                    if let Err(e) = self.map_overlay_pub.publish(&map) {
                        log_warn!(&self.nl, "Error publishing map_overlay: {}", e);
                    }
                    map_val += 10;
                    map_val %= 100;
                }
            }

            if let Err(e) = self.cmd_pub.publish(&twist) {
                log_warn!(&self.nl, "Error publishing twist: {}", e);
            }
        }
    }
}

impl Default for RosAgent {
    fn default() -> Self {
        Self::new()
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut agent = RosAgent::new();
    agent.run()
}
