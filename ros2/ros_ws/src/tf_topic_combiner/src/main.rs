use std::sync::{Arc, Mutex};

use futures::{future, StreamExt};
use r2r::QosProfile;
use r2r::{self, tf2_msgs};
use tokio::task;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Print args
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, "tf_topic_combiner", "")?;
    let arc_node = Arc::new(Mutex::new(node));

    let args = std::env::args().collect::<Vec<String>>();
    let n_robots = args
        .get(1)
        .expect("Provide number of robots")
        .parse::<u32>()
        .expect("Input is not a number");
    println!("Number of robots: {}", n_robots);

    for i in 0..n_robots {
        let robot_name = format!("robot_{}", i);
        let robot_name_cp = robot_name.clone();

        let topic_name = "tf".to_string();
        let node = arc_node.clone();
        let qos = QosProfile::default();
        println!("Relaying from /{robot_name}/{topic_name} to /{topic_name}",);
        task::spawn(async move {
            relay_tf_topic(node, robot_name_cp, topic_name, qos)
                .await
                .unwrap()
        });

        let qos = QosProfile::default().transient_local();
        let topic_name = "tf_static".to_string();
        println!("Relaying from /{robot_name}/{topic_name} to /{topic_name}",);
        let node = arc_node.clone();
        task::spawn(async move {
            relay_tf_topic(node, robot_name, topic_name, qos)
                .await
                .unwrap()
        });
    }

    let handle = tokio::task::spawn_blocking(move || loop {
        {
            arc_node
                .lock()
                .unwrap()
                .spin_once(std::time::Duration::from_millis(10));
        }
        std::thread::sleep(std::time::Duration::from_millis(100))
    });
    handle.await?;
    Ok(())
}

async fn relay_tf_topic(
    arc_node: Arc<Mutex<r2r::Node>>,
    robot_name: String,
    topic_name: String,
    qos: QosProfile,
) -> Result<(), r2r::Error> {
    let sub = arc_node
        .lock()
        .unwrap()
        .subscribe::<tf2_msgs::msg::TFMessage>(
            &format!("/{robot_name}/{topic_name}"),
            qos.clone(),
        )?;

    let publisher = arc_node
        .lock()
        .unwrap()
        .create_publisher::<tf2_msgs::msg::TFMessage>(&format!("/{topic_name}"), qos)?;
    let arc_publisher = Arc::new(Mutex::new(publisher));

    task::spawn(async move {
        sub.for_each(|mut msg| {
            msg.transforms.iter_mut().for_each(|transform| {
                transform.header.frame_id = format!("{robot_name}/{}", transform.header.frame_id);
                transform.child_frame_id = format!("{robot_name}/{}", transform.child_frame_id);
            });
            arc_publisher.lock().unwrap().publish(&msg).unwrap();
            future::ready(())
        })
        .await
    });
    Ok(())
}
