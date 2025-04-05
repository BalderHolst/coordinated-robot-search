use std::sync::{Arc, Mutex};

use agent::RosAgent;
use r2r::{log_info, log_warn};

mod agent;
mod convert_msg;
mod map_handler;
mod util;
mod vision;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "ros_agent", "")?;
    let nl = node.logger().to_string();

    if node.get_parameter::<bool>("use_sim_time").unwrap_or(false) {
        node.get_time_source()
            .enable_sim_time(&mut node)
            .expect("Could not use sim time");
        log_info!(&nl, "Using simulated time");
    } else {
        log_info!(&nl, "Using system time");
    }

    let node = Arc::new(Mutex::new(node));

    // Keep the node alive
    // No await since it should run forever
    tokio::task::spawn(util::ros2_node_spinner(node.clone()));

    let mut agent = tokio::spawn(RosAgent::new(node.clone())).await?;

    // Should not return from here
    agent.run().await?;
    log_warn!(&nl, "Ros Agent shutting down.");

    Ok(())
}
