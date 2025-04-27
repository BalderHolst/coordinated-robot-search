use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use botbrain::{MapCell, scaled_grid::ScaledGrid};
use futures::StreamExt;
use r2r::{QosProfile, log_info, nav_msgs};

pub async fn ros2_node_spinner(node: Arc<Mutex<r2r::Node>>) {
    let node_logger = node.lock().unwrap().logger().to_string();
    r2r::log_info!(&node_logger, "Node spinner started.");
    let duration = std::time::Duration::from_secs_f32(0.3);
    loop {
        let now = std::time::Instant::now();
        // Sleep with tokio::time::sleep instead of spin_once, since spin_once
        // blocks the current thread while still holding the lock.
        node.lock().unwrap().spin_once(Duration::ZERO);
        tokio::time::sleep(duration.saturating_sub(now.elapsed())).await;
    }
}

pub async fn spawn_ros2_subscriber<T: r2r::WrappedTypesupport + Send + Unpin + 'static>(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
    qos: QosProfile,
) -> Arc<Mutex<Option<T>>> {
    // Subscribe to topic
    let mut subscriber = node.lock().unwrap().subscribe::<T>(topic, qos).unwrap();
    let topic_msg = Arc::new(Mutex::new(None));
    {
        let nl = node.lock().unwrap().logger().to_string();
        let topic = topic.to_string();
        let topic_msg = topic_msg.clone();

        tokio::spawn(async move {
            log_info!(&nl, "{} listener started", topic);
            while let Some(msg) = subscriber.next().await {
                topic_msg.lock().unwrap().replace(msg);
            }
            log_info!(&nl, "{} listener broken", topic);
        });
    }
    topic_msg
}

pub fn get_ros2_time(node: Arc<Mutex<r2r::Node>>) -> Duration {
    node.lock()
        .unwrap()
        .get_ros_clock()
        .lock()
        .unwrap()
        .get_now()
        .unwrap()
}

pub fn ros2_map_to_botbrain_map(map: &nav_msgs::msg::OccupancyGrid) -> ScaledGrid<MapCell> {
    let (width, height) = (map.info.width, map.info.height);
    let scale = map.info.resolution;
    let mut botbrain_map = ScaledGrid::new(width as f32 * scale, height as f32 * scale, scale);

    for y in 0..height {
        for x in 0..width {
            let cell = map.data[y as usize * width as usize + x as usize];
            let cell = match cell {
                100 => MapCell::Obstacle,
                _ => MapCell::Free,
            };
            botbrain_map.grid_mut().set(x as usize, y as usize, cell);
        }
    }

    botbrain_map
}
