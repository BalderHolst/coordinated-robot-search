use std::path::PathBuf;

use robcore::{grid::Grid, scaled_grid::ScaledGrid, shapes::Shape};
use serde::{Deserialize, Serialize};

use crate::world::Cell;

use super::{pgm::PgmImage, World};

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

#[derive(Debug, Clone)]
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
                    grid.set(x, y, cell);
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
