use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use crate::{
    convert_msg::{
        agent_msg_to_ros2_msg, cov_pose_to_pose2d, ros2_msg_to_agent_msg, scan_to_lidar_data,
        sensor_image_to_opencv_image,
    },
    map_handler::Ros2MapHandler,
    util,
    vision::{camera_info::CameraInfo, object_detection::ObjectDetection},
};
use botbrain::{CamCone, CamData, Pos2, Vec2, debug_soup::DebugType, shapes::Cone};
use futures::StreamExt;
use opencv::highgui;
use r2r::{
    self, Publisher, QosProfile, geometry_msgs, log_error, log_info, log_warn, nav_msgs,
    ros_agent_msgs, sensor_msgs,
};

const DEFAULT_CHANNEL_TOPIC: &str = "/search_channel";

pub struct RosAgent {
    node: Arc<Mutex<r2r::Node>>,
    node_logger: String,

    robot: Box<dyn botbrain::Robot>,
    behavior: botbrain::behaviors::Behavior,

    pose: Arc<Mutex<Option<geometry_msgs::msg::PoseWithCovarianceStamped>>>,
    scan: Arc<Mutex<Option<sensor_msgs::msg::LaserScan>>>,
    cmd_pub: Publisher<geometry_msgs::msg::Twist>,

    map_overlay_pub: Publisher<nav_msgs::msg::OccupancyGrid>,
    map: Ros2MapHandler,

    image: Arc<Mutex<Option<sensor_msgs::msg::Image>>>,
    vision: ObjectDetection,

    incomming_msgs: Arc<Mutex<Vec<ros_agent_msgs::msg::AgentMessage>>>,
    outgoing_msgs_pub: Publisher<ros_agent_msgs::msg::AgentMessage>,
}

impl RosAgent {
    pub async fn new(node: Arc<Mutex<r2r::Node>>) -> Self {
        let node_logger = node.lock().unwrap().logger().to_string();

        log_info!(&node_logger, "Getting behavior");
        let Ok(behavior_param) = node.lock().unwrap().get_parameter::<String>("behavior") else {
            log_error!(
                &node_logger,
                "No `behavior` parameter provided. Please provide one. Possible values: [{}]",
                botbrain::behaviors::Behavior::behavior_names().join(", ")
            );
            std::process::exit(1);
        };

        log_info!(&node_logger, "Parsing behavior");
        let behavior = match botbrain::behaviors::Behavior::parse(&behavior_param) {
            Ok(behavior) => behavior,
            Err(e) => {
                log_error!(&node_logger, "Invalid `behavior` parameter provided: {}", e);
                std::process::exit(1);
            }
        };

        log_info!(&node_logger, "Constructing map");
        let map = tokio::spawn(Ros2MapHandler::new(node.clone()));

        log_info!(&node_logger, "Creating robot");
        let mut robot = behavior.create_robot();
        let id = node.lock().unwrap().get_parameter::<i64>("id");
        let id = match id {
            Ok(id) => id.try_into().ok(),
            Err(_) => (|| {
                // Try to parse id from node namespace
                let namespace = node.lock().unwrap().namespace().ok()?;
                let (_, end) = namespace.split_once("/robot_")?;
                end.parse().ok()
            })(),
        }
        .unwrap_or_default();
        log_info!(&node_logger, "Robot created");

        log_info!(&node_logger, "Robot ID: {}", id);
        robot.set_id(botbrain::RobotId::new(id));

        // Activate debug soup
        robot.get_debug_soup_mut().activate();

        let map = map.await.expect("Map handler failed");

        // Set world size
        let world = util::ros2_map_to_botbrain_map(&map.map);
        robot.set_map(world);

        // Publish to map_overlay
        let map_overlay_pub = node
            .lock()
            .unwrap()
            .create_publisher::<nav_msgs::msg::OccupancyGrid>(
                "map_overlay",
                QosProfile::default().volatile(),
            )
            .unwrap();

        // Subscribe to amcl_pose
        let pose = util::spawn_ros2_subscriber::<geometry_msgs::msg::PoseWithCovarianceStamped>(
            node.clone(),
            "amcl_pose",
            QosProfile::default().transient_local(),
        )
        .await;

        // Subscribe to laser scan
        let scan = util::spawn_ros2_subscriber::<sensor_msgs::msg::LaserScan>(
            node.clone(),
            "scan",
            QosProfile::sensor_data(),
        )
        .await;

        // Subscribe to image
        let image = util::spawn_ros2_subscriber::<sensor_msgs::msg::Image>(
            node.clone(),
            "rgbd_camera/image",
            QosProfile::default().keep_last(1),
        )
        .await;

        // Vertical FOV is not used
        // The params are directly set in the URDF for TB4
        let vision = ObjectDetection::new(CameraInfo::new(320, 240, 1.25, 1.0));

        // Publish to cmd_vel
        let cmd_pub = node
            .lock()
            .unwrap()
            .create_publisher::<geometry_msgs::msg::Twist>(
                "cmd_vel",
                QosProfile::default().volatile(),
            )
            .unwrap();

        // Subscribe/publish to the channel topic
        let channel_topic: String = node
            .lock()
            .unwrap()
            .get_parameter("channel_topic")
            .unwrap_or(DEFAULT_CHANNEL_TOPIC.into());

        let outgoing_msgs_pub = node
            .lock()
            .unwrap()
            .create_publisher::<ros_agent_msgs::msg::AgentMessage>(
                &channel_topic,
                QosProfile::default().volatile(),
            )
            .unwrap();

        let mut msgs_sub = node
            .lock()
            .unwrap()
            .subscribe::<ros_agent_msgs::msg::AgentMessage>(
                &channel_topic,
                QosProfile::default().volatile(),
            )
            .unwrap();

        let incomming_msgs = {
            let msgs = Arc::new(Mutex::new(Vec::new()));
            {
                let nl = node.lock().unwrap().logger().to_string();
                let msgs = msgs.clone();
                tokio::spawn(async move {
                    log_info!(&nl, "Message listener started on '{}'", channel_topic);
                    while let Some(msg) = msgs_sub.next().await {
                        msgs.lock().unwrap().push(msg);
                    }
                    log_info!(&nl, "Message listener broken");
                });
            }
            msgs
        };

        Self {
            node,
            node_logger,

            robot,
            behavior,

            pose,
            scan,
            cmd_pub,

            map,
            map_overlay_pub,

            image,
            vision,

            outgoing_msgs_pub,
            incomming_msgs,
        }
    }

    pub async fn run(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let mut last_map_update: Duration = util::get_ros2_time(self.node.clone());

        // Prevents ros_agent from updating before simulation is ready
        loop {
            let time = util::get_ros2_time(self.node.clone());

            if time.abs_diff(last_map_update).as_millis() >= 200 {
                // Simulated time is ready
                last_map_update = time;
                break;
            }

            // Simulated time is not available yet
            log_warn!(&self.node_logger, "RosTime not ready yet: {:?}", time);
            tokio::time::sleep(std::time::Duration::from_secs_f32(2.0)).await;
        }

        loop {
            tokio::time::sleep(std::time::Duration::from_secs_f32(0.05)).await;
            // log_info!(&self.node_logger, "Running agent");
            let time = util::get_ros2_time(self.node.clone());

            // Set inputs for robot
            {
                // Set the robot pose
                if let Some(pose) = self.pose.lock().unwrap().as_ref() {
                    let (pos, angle) = cov_pose_to_pose2d(pose);

                    let pos =
                        pos - self.map.map_origin - (self.map.map_size / 2.0) * self.map.map_scale;
                    self.robot.input_pose(botbrain::RobotPose { pos, angle });
                } else {
                    continue;
                }

                // Set the robot lidar input
                if let Some(scan) = self.scan.lock().unwrap().take() {
                    let lidar = scan_to_lidar_data(&scan);
                    self.robot.input_lidar(lidar);
                }

                // Set the object detection input
                if let Some(mut image) = self.image.lock().unwrap().take() {
                    if let Ok(img) = sensor_image_to_opencv_image(&mut image) {
                        match self.vision.find_search_objects_probability(&img) {
                            Ok(cam_data) => {
                                highgui::wait_key(1).unwrap();
                                // log_info!(&self.node_logger, "Found search objects:{:?}", cam_data);
                                self.robot.input_cam(cam_data);
                            }
                            Err(_) => {
                                let cam_data = CamData::Cone(CamCone {
                                    cone: {
                                        if let Some(pose) = self.pose.lock().unwrap().as_ref() {
                                            let pose = cov_pose_to_pose2d(pose);
                                            Cone {
                                                center: pose.0,
                                                radius: botbrain::params::CAM_RANGE, // TODO: Change this
                                                angle: pose.1,
                                                fov: 1.25,
                                            }
                                        } else {
                                            // log_error!(
                                            //     &self.node_logger,
                                            //     "No pose for updating object detection"
                                            // );
                                            Cone {
                                                center: Pos2::default(),
                                                radius: botbrain::params::CAM_RANGE, // TODO: Change this
                                                angle: 0.0,
                                                fov: 1.25,
                                            }
                                        }
                                    },
                                    probability: 0.0,
                                });
                                self.robot.input_cam(cam_data);
                            }
                        }
                    } else {
                        println!("Could not convert rgbd_image to opencv image");
                    }
                }

                // Set incomming messages
                {
                    let mut guard = self.incomming_msgs.lock().unwrap();
                    let msgs = guard.drain(..).filter_map(|ros_msg| {
                        let sender_id = ros_msg.sender_id;
                        match ros2_msg_to_agent_msg(ros_msg) {
                            Some(msg) => Some(msg),
                            None => {
                                log_warn!(
                                    &self.node_logger,
                                    "Error converting message from: {}",
                                    sender_id
                                );
                                None
                            }
                        }
                    });

                    // Deposit incoming msgs in the robot postbox
                    self.robot.input_msgs(msgs.collect());
                }
            }

            {
                let (control, outgoing_msgs) = (self.behavior.behavior_fn())(&mut self.robot, time);
                // Empty the outgoing messages from the robot postbox and send them to via ROS2
                outgoing_msgs.into_iter().for_each(|msg| {
                    if let Some(ros_msg) = agent_msg_to_ros2_msg(msg) {
                        self.outgoing_msgs_pub.publish(&ros_msg).unwrap();
                    } else {
                        log_warn!(&self.node_logger, "Error converting message to ROS2");
                    }
                });

                // Only linear x and angular z are used by robot
                let mut twist = geometry_msgs::msg::Twist::default();
                // Check for
                twist.linear.x = if control.speed.is_finite() {
                    control.speed as f64
                } else {
                    log_error!(&self.node_logger, "Wierd input x: {}", control.speed);
                    0.0
                };
                twist.angular.z = if control.steer.is_finite() {
                    control.steer as f64
                } else {
                    log_error!(&self.node_logger, "Wierd input z: {}", control.steer);
                    0.0
                };

                // Publish the cmd_vel
                if let Err(e) = self.cmd_pub.publish(&twist) {
                    log_warn!(&self.node_logger, "Error publishing twist: {}", e);
                }
            }

            if time > last_map_update + Duration::from_secs_f32(0.5) {
                last_map_update = time;
                self.update_map_overlay();
            }
        }
    }

    fn update_map_overlay(&self) {
        let mut map = self.map.map.clone();
        let search_grid = self.robot.get_debug_soup().get("Grids", "Costmap Grid");
        // let search_grid = self.robot.get_debug_soup().get("Grids", "Search Grid");
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        if let Some(item) = search_grid {
            if let DebugType::Grid(grid) = &*item {
                let (x_size, y_size) = (map.info.width, map.info.height);
                for y in 0..y_size {
                    for x in 0..x_size {
                        let cell = grid.get(
                            Pos2 {
                                x: x as f32 * self.map.map_scale,
                                y: y as f32 * self.map.map_scale,
                            } - grid.size() / 2.0,
                        );

                        if let Some(&cell) = cell {
                            min = min.min(cell);
                            max = max.max(cell);
                        } else {
                            log_error!(&self.node_logger, "Cell is none: {},{}", x, y);
                        }
                    }
                }
                for y in 0..y_size {
                    for x in 0..x_size {
                        let cell = grid.get(
                            Pos2 {
                                x: x as f32 * self.map.map_scale,
                                y: y as f32 * self.map.map_scale,
                            } - grid.size() / 2.0,
                        );
                        // Between 1 and 98 from blue to red
                        // 0 is black
                        let value: i8 = match cell {
                            Some(&cell) if cell > 0.0 => {
                                // Max should be 98
                                // 0 should be (98+1)/2
                                // Range (98-1)/2
                                let ratio = cell / max;
                                (49.5 + 48.5 * ratio).clamp(49.0, 98.0) as i8
                            }
                            Some(&cell) if cell < 0.0 => {
                                let ratio = cell / min;
                                (49.5 - 48.5 * ratio).clamp(1.0, 49.5) as i8
                            }
                            _ => 0,
                        };
                        map.data[y as usize * x_size as usize + x as usize] = value;
                    }
                }
            }
        } else {
            log_error!(&self.node_logger, "Map overlay is None");
        }

        if let Err(e) = self.map_overlay_pub.publish(&map) {
            log_warn!(&self.node_logger, "Error publishing map_overlay: {}", e);
        }
    }
}
