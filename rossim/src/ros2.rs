use futures::{executor::LocalPool, task::LocalSpawnExt, StreamExt};
use r2r::QosProfile;
use robcore::Pos2;

use crate::world::{self, Cell, World};

pub struct Ros2 {
    node: r2r::Node,
    map: Option<r2r::nav_msgs::msg::OccupancyGrid>,
    pool: LocalPool,
}

impl Ros2 {
    fn new() -> Self {
        let ctx = r2r::Context::create().unwrap();
        let mut node = r2r::Node::create(ctx, "echolist", "").unwrap();
        let qos = QosProfile::default()
            .reliable()
            .transient_local()
            .keep_all();
        let mut sub = node
            .subscribe::<r2r::nav_msgs::msg::OccupancyGrid>("/map", qos)
            .unwrap();

        let pool = LocalPool::new();

        // task that every other time forwards message to topic2
        pool.spawner()
            .spawn_local(async move {
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
                        println!(
                            "Map info: {},{}:{}",
                            msg.info.height, msg.info.width, msg.info.resolution
                        );
                    } else {
                        println!("Broken");
                        break;
                    }
                }
            })
            .unwrap();
        Self {
            node,
            map: None,
            pool,
        }
    }
    fn to_world(&self) -> Option<world::World> {
        if let Some(map) = &self.map {
            let width = map.info.width as f32;
            let height = map.info.height as f32;
            let mut world = World::new(width, height);
            for x in 0..width as usize {
                for y in 0..height as usize {
                    let cell = match map.data[y as usize * width as usize + x as usize] {
                        0 => Cell::Wall,
                        100 => Cell::Empty,
                        -1 => Cell::OutOfBounds,
                        _ => Cell::OutOfBounds,
                    };
                    world.set_cell(Pos2::new(x as f32, y as f32), cell);
                }
            }
            Some(world)
        } else {
            None
        }
    }
}
