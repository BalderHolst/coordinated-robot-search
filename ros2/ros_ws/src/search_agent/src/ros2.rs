use std::{
    borrow::BorrowMut,
    sync::{Arc, Mutex},
};

use futures::{executor::LocalPool, task::LocalSpawnExt, StreamExt};
use r2r::{sensor_msgs, QosProfile};
use robcore::{self, LidarData, LidarPoint};

// use crate::world::{self, Cell, World};

pub struct Ros2 {
    pub node: r2r::Node,
    pub pool: LocalPool,
    // pub transform: Arc<Mutex<Option<r2r::geometry_msgs::msg::Pose2D>>>,
    pub scan: Arc<Mutex<Option<sensor_msgs::msg::LaserScan>>>,
    // pub map: Arc<Mutex<Option<r2r::nav_msgs::msg::OccupancyGrid>>>,
    // had_map_update: Arc<Mutex<bool>>,
}

impl Ros2 {
    pub fn new() -> Self {
        let ctx = r2r::Context::create().unwrap();
        let mut node = r2r::Node::create(ctx, "agent", "robot_0").unwrap(); // Namespace set from launch
                                                                            // file

        let qos = QosProfile::default().volatile();
        let mut sub_scan = node
            .subscribe::<sensor_msgs::msg::LaserScan>("scan", qos)
            .unwrap();

        let scan = Arc::new(Mutex::new(None));
        let pool = LocalPool::new();
        {
            let scan = scan.clone();
            println!("Pool created");

            pool.spawner()
                .spawn_local(async move {
                    println!("Agent node started");
                    loop {
                        if let Some(msg) = sub_scan.next().await {
                            println!("We got data!");
                            scan.lock().unwrap().replace(msg);
                        } else {
                            println!("Broken");
                            break;
                        }
                    }
                })
                .unwrap();
        }
        Self { node, pool, scan }
    }
}
// pub struct LaserScan {
//     pub header: std_msgs::msg::Header,
//     pub angle_min: f32,
//     pub angle_max: f32,
//     pub angle_increment: f32,
//     pub time_increment: f32,
//     pub scan_time: f32,
//     pub range_min: f32,
//     pub range_max: f32,
//     pub ranges: Vec<f32>,
//     pub intensities: Vec<f32>,
// }
pub fn scan_to_lidar_data(scan: &sensor_msgs::msg::LaserScan) -> robcore::LidarData {
    let mut lidar_data = Vec::with_capacity(scan.ranges.len());
    for (i, rng) in scan.ranges.iter().enumerate() {
        let angle = scan.angle_min + scan.angle_increment * i as f32;
        lidar_data.push(LidarPoint {
            angle,
            distance: *rng,
        });
    }
    LidarData(lidar_data)
}
