use std::sync::{Arc, Mutex};

use botbrain::Vec2;
use futures::StreamExt;
use r2r::{QosProfile, log_error, log_info, nav_msgs};

pub struct Ros2MapHandler {
    _node: Arc<Mutex<r2r::Node>>,
    pub map: nav_msgs::msg::OccupancyGrid,
    pub map_origin: Vec2,
    pub map_scale: f32,
    pub map_size: Vec2,
}

impl Ros2MapHandler {
    pub async fn new(node: Arc<Mutex<r2r::Node>>) -> Self {
        let node_logger = node.lock().unwrap().logger().to_string();

        let map = {
            let node_logger = node_logger.clone();

            // Subscribe to map
            let mut sub_map = node
                .lock()
                .unwrap()
                .subscribe::<nav_msgs::msg::OccupancyGrid>(
                    "/map",
                    QosProfile::default().transient_local(),
                )
                .unwrap();

            tokio::spawn(async move {
                log_info!(&node_logger, "Map listener started");
                // No need wor while let as we only need the map once
                if let Some(msg) = sub_map.next().await {
                    log_info!(&node_logger, "Map recieved");
                    Some(msg)
                } else {
                    log_error!(&node_logger, "Map subscriber broken");
                    None
                }
            })
        };

        let map = map.await.unwrap().expect("Map subscriber broken");

        let map_scale = map.info.resolution;

        let map_origin = Vec2::new(
            map.info.origin.position.x as f32,
            map.info.origin.position.y as f32,
        );

        let map_size = Vec2::new(map.info.width as f32, map.info.height as f32);

        Self {
            _node: node,
            map,
            map_origin,
            map_scale,
            map_size,
        }
    }
}
