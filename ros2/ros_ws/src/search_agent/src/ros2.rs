use std::{
    borrow::BorrowMut,
    sync::{Arc, Mutex},
};

use futures::{executor::LocalPool, task::LocalSpawnExt, StreamExt};
use r2r::{geometry_msgs, search_agent_msgs, sensor_msgs, Publisher, QosProfile};
use robcore::{self, LidarData, LidarPoint, RobotId};

const DEFAULT_CHANNEL_TOPIC: &str = "/search_channel";

pub struct Ros2 {
    pub node: r2r::Node,
    pub pool: LocalPool,
    pub pose: Arc<Mutex<Option<geometry_msgs::msg::PoseWithCovarianceStamped>>>,
    pub scan: Arc<Mutex<Option<sensor_msgs::msg::LaserScan>>>,
    pub incomming_msgs: Arc<Mutex<Vec<search_agent_msgs::msg::AgentMessage>>>,
    pub outgoing_msgs_pub: Publisher<search_agent_msgs::msg::AgentMessage>,
    // pub map: Arc<Mutex<Option<r2r::nav_msgs::msg::OccupancyGrid>>>,
    // had_map_update: Arc<Mutex<bool>>,
}

impl Ros2 {
    pub fn new() -> Self {
        let ctx = r2r::Context::create().unwrap();

        let mut node = r2r::Node::create(ctx, "agent", "").unwrap();

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

        let scan = Arc::new(Mutex::new(None));
        let pose = Arc::new(Mutex::new(None));
        let incomming_msgs = Arc::new(Mutex::new(Vec::new()));

        let pool = LocalPool::new();
        {
            let scan = scan.clone();
            println!("Pool created");

            pool.spawner()
                .spawn_local(async move {
                    println!("Scan listener started");
                    while let Some(msg) = sub_scan.next().await {
                        scan.lock().unwrap().replace(msg);
                    }
                    println!("Scan listener broken");
                })
                .unwrap();

            let pose = pose.clone();
            pool.spawner()
                .spawn_local(async move {
                    println!("Pose listener started");
                    while let Some(msg) = sub_pose.next().await {
                        pose.lock().unwrap().replace(msg);
                    }
                })
                .unwrap();

            let incomming_msgs = incomming_msgs.clone();
            pool.spawner()
                .spawn_local(async move {
                    println!("Message listener started");
                    while let Some(msg) = msgs_sub.next().await {
                        incomming_msgs.lock().unwrap().push(msg);
                    }
                    println!("Message listener broken");
                })
                .unwrap();
        }
        Self {
            node,
            pool,
            scan,
            pose,
            incomming_msgs,
            outgoing_msgs_pub: msgs_pub,
        }
    }
}

pub fn scan_to_lidar_data(scan: &sensor_msgs::msg::LaserScan) -> robcore::LidarData {
    let mut lidar_data = Vec::with_capacity(scan.ranges.len());
    for (i, rng) in scan.ranges.iter().enumerate() {
        let angle = scan.angle_min + scan.angle_increment * i as f32;
        lidar_data.push(LidarPoint {
            angle,
            distance: *rng,
        });
    }
    LidarData::new(lidar_data)
}

pub fn cov_pose_to_pose2d(
    pose: &r2r::geometry_msgs::msg::PoseWithCovarianceStamped,
) -> (robcore::Pos2, f32) {
    let pos = &pose.pose.pose.position;
    let pos: robcore::Pos2 = robcore::Pos2::new(pos.x as f32, pos.y as f32);
    let geometry_msgs::msg::Quaternion { x, y, z, w } = &pose.pose.pose.orientation;
    let angle = f64::atan2(2. * w * z + x * y, 1. - 2. * (x.powi(2) * y.powi(2)));
    (pos, angle as f32)
}

pub fn ros2_msg_to_agent_msg(
    msg: search_agent_msgs::msg::AgentMessage,
) -> Option<robcore::Message> {
    Some(robcore::Message {
        sender_id: RobotId::new(msg.sender_id),
        kind: msg.data.try_into().ok()?,
    })
}

pub fn agent_msg_to_ros2_msg(
    msg: robcore::Message,
) -> Option<search_agent_msgs::msg::AgentMessage> {
    Some(search_agent_msgs::msg::AgentMessage {
        sender_id: msg.sender_id.as_u32(),
        data: msg.kind.try_into().ok()?,
    })
}
