use futures::{executor::LocalPool, future, stream::StreamExt, task::LocalSpawnExt};
use r2r::nav_msgs::msg::OccupancyGrid;
use r2r::QosProfile;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "map2grid", "")?;
    let subscriber = node.subscribe::<OccupancyGrid>("/map", QosProfile::default())?;

    println!("node name: {}", node.name()?);
    println!(
        "node fully qualified name: {}",
        node.fully_qualified_name()?
    );
    // Set up a simple task executor.
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // Run the subscriber in one task, printing the messages
    spawner.spawn_local(async move {
        subscriber
            .for_each(|msg| {
                println!("got new msg: {:?}", msg.data);
                future::ready(())
            })
            .await
    })?;

    // Main loop spins ros.
    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
