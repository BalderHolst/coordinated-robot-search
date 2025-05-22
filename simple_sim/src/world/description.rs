use std::path::PathBuf;

use botbrain::{grid::Grid, scaled_grid::ScaledGrid, shapes::Shape};
use serde::{Deserialize, Serialize};

use crate::world::Cell;

use super::{pgm::PgmImage, World};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObjectDescription {
    pub width: f32,
    pub height: f32,
    pub scale: f32,
    #[serde(default)]
    pub obstacles: Vec<Shape>,
    #[serde(default)]
    pub search_items: Vec<Shape>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BitmapDescriptionMode {
    Trinary,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BitmapDescription {
    pub image: PathBuf,
    pub mode: BitmapDescriptionMode,
    pub resolution: f32,
    pub origin: [f32; 3],
    pub negate: i32,
    pub occupied_thresh: f32,
    pub free_thresh: f32,
    #[serde(skip)]
    pub bitmap: PgmImage,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum WorldDescription {
    Bitmap(BitmapDescription),
    Objs(ObjectDescription),
}

impl WorldDescription {
    pub fn create(self) -> World {
        match self {
            WorldDescription::Bitmap(BitmapDescription {
                resolution,
                bitmap: image,
                ..
            }) => {
                let mut grid = Grid::new(image.width, image.height);
                for (x, y, value) in image.iter() {
                    let cell = match value {
                        0 | 205 => Cell::Wall,
                        254 => Cell::Empty,
                        other => {
                            eprintln!("Invalid value in PGM image: {}", other);
                            Cell::Empty
                        }
                    };
                    grid.set(x, image.height - y, cell);
                }

                ScaledGrid::from_grid(grid, resolution)
            }
            WorldDescription::Objs(ObjectDescription {
                width,
                height,
                scale,
                obstacles,
                search_items,
            }) => {
                let mut world = World::new(width, height, scale);
                for shape in obstacles {
                    world.set_shape(&shape, Cell::Wall);
                }
                for search_item in search_items {
                    world.set_shape(&search_item, Cell::SearchItem);
                }
                world
            }
        }
    }
}
