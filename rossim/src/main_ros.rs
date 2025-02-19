use std::time::Duration;

mod ros2;

fn main() {
    let mut ros2 = ros2::Ros2::new();
    loop {
        ros2.node.spin_once(Duration::from_secs(0));
        ros2.pool.run_until_stalled();

        if ros2.had_map_update() {
            let map = ros2.map.lock().unwrap();
            if let Some(map) = map.as_ref() {
                println!("{:?}", map.header);
                println!("{:?}\n\n", map.info);
                // println!("{:?}", map.data);
            }
        }
    }
}
