use opencv::highgui;
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use futures::{StreamExt, executor::LocalPool, task::LocalSpawnExt};
use r2r::{self, QosProfile, log_info, sensor_msgs};

use crate::camera::Camera;

pub struct RosAgent {
    node: r2r::Node,
    nl: String,
    pool: LocalPool,
    image: Arc<Mutex<Option<sensor_msgs::msg::Image>>>,
    camera: Camera,
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
            .subscribe::<sensor_msgs::msg::Image>(
                "rgbd_camera/image",
                QosProfile::default().keep_last(1),
            )
            .unwrap();
        {
            let nl = nl.clone();
            let image = image.clone();
            pool.spawner()
                .spawn_local(async move {
                    log_info!(&nl, "Image listener started");
                    while let Some(msg) = sub_image.next().await {
                        image.lock().unwrap().replace(msg);
                    }
                })
                .unwrap();
        }
        let camera = Camera::new(320, 240, 1.25, 1.0); // Vertical
        // fov not used

        Self {
            node,
            nl,
            pool,
            image,
            camera,
        }
    }

    pub fn run(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        loop {
            self.node.spin_once(Duration::from_secs(1));
            self.pool.run_until_stalled();
            if let Some(mut image) = self.image.lock().unwrap().take() {
                if let Ok(img) = crate::camera::sensor_image_to_opencv_image(&mut image) {
                    highgui::imshow("robot_0", &img).unwrap();
                    highgui::wait_key(1).unwrap();
                } else {
                    println!("Could not convert image to opencv image");
                }
            }
        }
    }
}

impl Default for RosAgent {
    fn default() -> Self {
        Self::new()
    }
}
