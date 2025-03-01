use std::{
    borrow::BorrowMut,
    sync::{Arc, Mutex},
};

use futures::{executor::LocalPool, task::LocalSpawnExt, StreamExt};
use r2r::QosProfile;
// use robcore::Pos2;

// use crate::world::{self, Cell, World};

pub struct Ros2 {
    pub node: r2r::Node,
    pub pool: LocalPool,
    pub map: Arc<Mutex<Option<r2r::nav_msgs::msg::OccupancyGrid>>>,
    had_map_update: Arc<Mutex<bool>>,
}

impl Ros2 {
    pub fn new() -> Self {
        let ctx = r2r::Context::create().unwrap();
        let mut node = r2r::Node::create(ctx, "echolist", "").unwrap();
        let qos = QosProfile::default()
            .reliable()
            .transient_local()
            .keep_all();
        let mut sub = node
            .subscribe::<r2r::nav_msgs::msg::OccupancyGrid>("/map", qos)
            .unwrap();

        let map = Arc::new(Mutex::new(None));
        let map_cp = Arc::clone(&map);
        let had_map_update = Arc::new(Mutex::new(false));
        let had_map_update_cp = Arc::clone(&had_map_update);

        let pool = LocalPool::new();
        println!("Pool created");

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
                        let mut map_updated = had_map_update_cp.lock().unwrap();
                        *map_updated = true;

                        println!(
                            "Map info: {},{}:{}",
                            msg.info.height, msg.info.width, msg.info.resolution
                        );
                        map_cp.lock().unwrap().replace(msg);
                    } else {
                        println!("Broken");
                        break;
                    }
                }
            })
            .unwrap();
        Self {
            node,
            pool,
            map,
            had_map_update,
        }
    }

    pub fn had_map_update(&self) -> bool {
        if let Ok(mut map_updated) = self.had_map_update.lock() {
            let is_updated = *map_updated;
            match is_updated {
                true => {
                    **map_updated.borrow_mut() = false;
                    true
                }
                false => false,
            }
        } else {
            false
        }
    }
    // fn map_to_world(&self) -> Option<world::World> {
    //     let map = self.map.lock().unwrap();
    //     if let Some(map) = map.as_ref() {
    //         let width = map.info.width;
    //         let height = map.info.height;
    //         let mut world = World::new(width as f32, height as f32);
    //         for x in 0..width {
    //             for y in 0..height {
    //                 let cell = match map.data[(y * width + x) as usize] {
    //                     0 => Cell::Empty,
    //                     100 => Cell::Wall,
    //                     -1 => Cell::OutOfBounds,
    //                     _ => Cell::OutOfBounds,
    //                 };
    //                 world.set_cell(Pos2::new(x as f32, y as f32), cell);
    //             }
    //         }
    //         Some(world)
    //     } else {
    //         None
    //     }
    // }
}
