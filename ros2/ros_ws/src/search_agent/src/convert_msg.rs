use r2r::{geometry_msgs, search_agent_msgs, sensor_msgs};
use botbrain::{self, LidarData, LidarPoint, RobotId};

pub fn scan_to_lidar_data(scan: &sensor_msgs::msg::LaserScan) -> botbrain::LidarData {
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
) -> (botbrain::Pos2, f32) {
    let pos = &pose.pose.pose.position;
    let pos: botbrain::Pos2 = botbrain::Pos2::new(pos.x as f32, pos.y as f32);
    let geometry_msgs::msg::Quaternion { x, y, z, w } = &pose.pose.pose.orientation;
    let angle = f64::atan2(2. * w * z + x * y, 1. - 2. * (x.powi(2) * y.powi(2)));
    (pos, angle as f32)
}

pub fn ros2_msg_to_agent_msg(
    msg: search_agent_msgs::msg::AgentMessage,
) -> Option<botbrain::Message> {
    Some(botbrain::Message {
        sender_id: RobotId::new(msg.sender_id),
        kind: msg.data.try_into().ok()?,
    })
}

pub fn agent_msg_to_ros2_msg(
    msg: botbrain::Message,
) -> Option<search_agent_msgs::msg::AgentMessage> {
    Some(search_agent_msgs::msg::AgentMessage {
        sender_id: msg.sender_id.as_u32(),
        data: msg.kind.try_into().ok()?,
    })
}
