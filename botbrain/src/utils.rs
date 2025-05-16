//! Utility functions for the botbrain library

use std::f32::consts::PI;

use emath::Pos2;

use crate::{params, scaled_grid::ScaledGrid, Map, MapCell, RobotPose};

/// Normalize an angle to be in the range [-PI, PI]
pub fn normalize_angle(angle: f32) -> f32 {
    let mut angle = angle;
    while angle < -PI {
        angle += 2.0 * PI;
    }
    while angle > PI {
        angle -= 2.0 * PI;
    }
    angle
}

/// A struct used to calculate map coverage for benchmarking search algorithms
#[derive(Clone)]
pub struct CoverageGrid {
    coverage_grid: ScaledGrid<bool>,
    obstacle_coverage: f32,
    coverage: f32,
    map: Map, // TODO: Use this to infer lidar
}

impl CoverageGrid {
    const CELL_SIZE: f32 = 0.25;

    /// Create a new coverage grid with the given dimensions
    pub fn new(map: Map) -> Self {
        let mut coverage_grid = ScaledGrid::<bool>::new(map.width(), map.height(), Self::CELL_SIZE);

        let mut obstacle_cells: usize = 0;
        for (x, y, covered) in coverage_grid.iter_mut() {
            if !*covered && matches!(map.get(Pos2::new(x, y)), Some(MapCell::Obstacle)) {
                *covered = true;
                obstacle_cells += 1;
            }
        }

        let obstacle_coverage = obstacle_cells as f32 * Self::CELL_SIZE.powi(2);

        Self {
            coverage_grid,
            obstacle_coverage,
            coverage: 0.0,
            map,
        }
    }

    fn area(&self) -> f32 {
        self.coverage_grid.width() * self.coverage_grid.height() - self.obstacle_coverage
    }

    /// Mark a pose as covered
    pub fn mark_pose(&mut self, pose: RobotPose) {
        {
            let area = self.area();
            let center = self.coverage_grid.pos_to_grid(pose.pos);
            let radius = params::CAM_RANGE / Self::CELL_SIZE;
            let angle = pose.angle;
            let fov = params::CAM_FOV;

            let inner_grid = self.coverage_grid.grid_mut();
            for (x, y) in inner_grid.iter_cone(center, radius, angle, fov) {
                let Some(cell) = inner_grid.get_mut(x, y) else {
                    continue;
                };
                if !*cell {
                    self.coverage += Self::CELL_SIZE * Self::CELL_SIZE / area;
                }
                *cell = true;
            }
        }
    }

    /// Get the current coverage in range [0, 1].
    pub fn coverage(&self) -> f32 {
        self.coverage.min(1.0)
    }

    /// Get the grid used to track coverage
    pub fn grid(&self) -> &ScaledGrid<bool> {
        &self.coverage_grid
    }

    /// Get the map
    pub fn map(&self) -> &Map {
        &self.map
    }
}
