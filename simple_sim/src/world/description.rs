use robcore::shapes::Shape;
use serde::{Deserialize, Serialize};

use crate::world::Cell;

use super::World;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObjectDescription {
    width: f32,
    height: f32,
    scale: f32,
    #[serde(default)]
    obstacles: Vec<Shape>,
    #[serde(default)]
    search_items: Vec<Shape>,
}

#[derive(Debug, Clone)]
pub struct BitmapDescription {
    width: u32,
    height: u32,
    data: Vec<u8>,
}

#[derive(Debug, Clone)]
pub enum WorldDescription {
    Bitmap(BitmapDescription),
    Objs(ObjectDescription),
}

impl WorldDescription {
    pub fn create(self) -> World {
        match self {
            WorldDescription::Bitmap(bitmap_description) => todo!(),
            WorldDescription::Objs(ObjectDescription {
                width,
                height,
                scale,
                obstacles,
                search_items,
            }) => {
                let mut world = World::new(width, height, scale);
                for shape in obstacles {
                    world.shape(&shape, Cell::Wall);
                }
                for search_item in search_items {
                    world.shape(&search_item, Cell::SearchItem);
                }
                world
            }
        }
    }
}
