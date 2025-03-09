use std::{
    borrow::BorrowMut,
    sync::{Arc, Mutex},
};

use futures::{executor::LocalPool, task::LocalSpawnExt, StreamExt};
use r2r::{geometry_msgs, sensor_msgs, QosProfile};
use robcore::{self, LidarData, LidarPoint};

// use crate::world::{self, Cell, World};

pub struct Ros2 {
    pub node: r2r::Node,
    pub pool: LocalPool,
    pub pose: Arc<Mutex<Option<r2r::geometry_msgs::msg::PoseWithCovarianceStamped>>>,
    pub scan: Arc<Mutex<Option<sensor_msgs::msg::LaserScan>>>,
    // pub map: Arc<Mutex<Option<r2r::nav_msgs::msg::OccupancyGrid>>>,
    // had_map_update: Arc<Mutex<bool>>,
}

impl Ros2 {
    pub fn new() -> Self {
        let ctx = r2r::Context::create().unwrap();
        let mut node = r2r::Node::create(ctx, "agent", "").unwrap(); // Namespace set from launch
                                                                     // file

        let qos = QosProfile::default().volatile();
        let mut sub_scan = node
            .subscribe::<sensor_msgs::msg::LaserScan>("scan", qos)
            .unwrap();

        let qos = QosProfile::default().transient_local();
        let mut sub_pose = node
            .subscribe::<r2r::geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", qos)
            .unwrap();

        let scan = Arc::new(Mutex::new(None));
        let pose = Arc::new(Mutex::new(None));

        let pool = LocalPool::new();
        {
            let scan = scan.clone();
            println!("Pool created");

            pool.spawner()
                .spawn_local(async move {
                    println!("Scan listener started");
                    loop {
                        if let Some(msg) = sub_scan.next().await {
                            scan.lock().unwrap().replace(msg);
                        } else {
                            println!("Broken");
                            break;
                        }
                    }
                })
                .unwrap();

            let pose = pose.clone();
            pool.spawner()
                .spawn_local(async move {
                    println!("Pose listener started");
                    loop {
                        if let Some(msg) = sub_pose.next().await {
                            // println!("Pose received");
                            pose.lock().unwrap().replace(msg);
                        } else {
                            println!("Broken");
                            break;
                        }
                    }
                })
                .unwrap();
        }
        Self {
            node,
            pool,
            scan,
            pose,
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
