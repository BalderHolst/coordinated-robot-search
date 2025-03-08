use futures::{executor::LocalPool, task::LocalSpawnExt, StreamExt};
use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

const DEFAULT_TOPIC: &str = "/search_channel";

use futures::task::SpawnExt;
use r2r::{self, search_agent_msgs, QosProfile};
use botbrain::MessageKind;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create().unwrap();
    let mut node = r2r::Node::create(ctx, "agent", "").unwrap();
    let logger = node.logger().to_string().leak();

    let topic: String = node.get_parameter("topic").unwrap_or(DEFAULT_TOPIC.into());

    let qos = QosProfile::default().volatile();
    let mut sub = node
        .subscribe::<search_agent_msgs::msg::AgentMessage>(&topic, qos)
        .unwrap();

    let mut pool = LocalPool::new();
    pool.spawner()
        .spawn_local(async move {
            r2r::log_info!(logger, "Printing messages over topic: {}", topic);
            while let Some(msg) = sub.next().await {
                (|msg: search_agent_msgs::msg::AgentMessage| match MessageKind::try_from(msg.data) {
                    Ok(kind) => {
                        match kind {
                            MessageKind::ShapeDiff { shape, diff } => {} // TODO: Print shape diff
                            MessageKind::CamDiff { cone, lidar, diff } => {} // TODO: Print cam diff
                            MessageKind::Debug(s) => {
                                r2r::log_info!(logger, "[{}] Debug: {}", msg.sender_id, s)
                            },
                        }
                    }
                    Err(e) => r2r::log_error!(logger, "[{}] Corrupt message: {}", msg.sender_id, e),
                })(msg);
            }
        })
        .unwrap();

    loop {
        node.spin_once(Duration::from_secs(1));
        pool.run_until_stalled();
    }
}
