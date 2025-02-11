use futures::{executor::LocalPool, stream::StreamExt, task::LocalSpawnExt};
use r2r::QosProfile;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    r2r::log_info!("before_init", "debug msg");
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "echolist", "")?;
    let qos = QosProfile::default()
        .reliable()
        .transient_local()
        .keep_all();
    let mut sub = node.subscribe::<r2r::nav_msgs::msg::OccupancyGrid>("/map", qos.clone())?;
    // let mut p = node.create_publisher::<r2r::nav_msgs::msg::OccupancyGrid>("/map", qos)?;
    let nl = node.logger().to_owned();

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // task that every other time forwards message to topic2
    spawner.spawn_local(async move {
        println!("Echo list node started");
        loop {
            // Code to publish to a topic
            // let send = r2r::nav_msgs::msg::OccupancyGrid {
            //     info: r2r::nav_msgs::msg::MapMetaData {
            //         resolution: 5.25,
            //         ..Default::default()
            //     },
            //     ..Default::default()
            // };
            // p.publish(&send).unwrap();
            if let Some(msg) = sub.next().await {
                r2r::log_info!(
                    &nl,
                    "Map info: {},{}:{}",
                    msg.info.height,
                    msg.info.width,
                    msg.info.resolution
                );
                r2r::log_debug!(&nl, "test");
                r2r::log_info!(&nl, "test");
                r2r::log_warn!(&nl, "test");
                r2r::log_error!(&nl, "test");
                r2r::log_fatal!(&nl, "test");
            } else {
                r2r::log_fatal!(&nl, "broken");
                break;
            }
        }
    })?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
