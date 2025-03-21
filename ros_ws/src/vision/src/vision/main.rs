use opencv::prelude::*;
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use futures::{StreamExt, executor::LocalPool, task::LocalSpawnExt};
use r2r::{self, QosProfile, log_info, sensor_msgs};

pub struct RosAgent {
    node: r2r::Node,
    nl: String,
    pool: LocalPool,
    image: Arc<Mutex<Option<sensor_msgs::msg::Image>>>,
}

impl RosAgent {
    pub fn new() -> Self {
        let ctx = r2r::Context::create().unwrap();

        let mut node = r2r::Node::create(ctx, "vision", "robot_0").unwrap();

        let nl = node.logger().to_string();

        let pool = LocalPool::new();

        // Subscribe to map
        let image = Arc::new(Mutex::new(None));
        let mut sub_image = node
            .subscribe::<sensor_msgs::msg::Image>("rgbd_image", QosProfile::default())
            .unwrap();
        {
            let nl = nl.clone();
            let image = image.clone();
            pool.spawner()
                .spawn_local(async move {
                    log_info!(&nl, "Map listener started");
                    while let Some(msg) = sub_image.next().await {
                        image.lock().unwrap().replace(msg);
                        log_info!(&nl, "Map received");
                    }
                })
                .unwrap();
        }

        Self {
            node,
            nl,
            pool,
            image,
        }
    }

    pub fn run(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        loop {
            self.node.spin_once(Duration::from_secs(1));
            self.pool.run_until_stalled();
            if let Some(image) = self.image.lock().unwrap().take() {
                println!("Image received");
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
