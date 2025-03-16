use std::time::Duration;

#[cfg(r2r__rosgraph_msgs__msg__Clock)]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "ros2time", "")?;
    if node.get_parameter::<bool>("use_sim_time").unwrap_or(false) {
        node.get_time_source()
            .enable_sim_time(&mut node)
            .expect("Could not use sim time");
    }
    loop {
        {
            let clock = node.get_ros_clock();
            let now = clock.lock().unwrap().get_now()?;
            if !now.is_zero() {
                println!("nodetime: {:?}", now);
            }
        }
        {
            let mut clock = r2r::Clock::create(r2r::ClockType::SystemTime)?;
            let now = clock.get_now()?;
            println!("systemtime: {:?}", now);
        }
        {
            let mut clock = r2r::Clock::create(r2r::ClockType::RosTime)?;
            let now = clock.get_now()?;
            println!("rostime: {:?}", now);
        }
        println!();
        std::thread::sleep(std::time::Duration::from_secs_f32(0.1));
        node.spin_once(Duration::from_secs_f32(0.1));
    }
}

#[cfg(not(r2r__rosgraph_msgs__msg__Clock))]
fn main() {
    panic!("Error not compiled with 'rosgraph_msgs'.");
}
