//! This module contains robot `search` behavior.

use costmap::COSTMAP_GRID_SCALE;
use pathing::PATH_PLANNER_DISTANCE_TOLERANCE;
use std::{
    collections::{HashMap, HashSet},
    f32::consts::PI,
    ops::Range,
    time::Duration,
};

use emath::{Pos2, Vec2};

use crate::{params::COMMUNICATION_RANGE, LidarData};

use super::{
    cast_robot, common, debug_soup,
    params::{self},
    scaled_grid::ScaledGrid,
    shapes::{Circle, Line},
    utils::normalize_angle,
    BehaviorFn, BehaviorOutput, CamData, Control, DebugItem, DebugSoup, Map, Postbox, Robot,
    RobotId, RobotPose, RobotRef,
};

mod costmap;
mod frontiers;
mod pathing;

pub const MENU: &[(&str, BehaviorFn)] = &[
    ("full", behaviors::full),
    ("naive-proximity", behaviors::naive_proximity),
    ("no-proximity", behaviors::no_proximity),
    ("no-pathing", behaviors::no_pathing),
    ("pure-pathing", behaviors::pure_pathing),
    // Experimental to test frontier exploration params
    // s is size weight
    // d is distance weight
    // t is turn weight
    // Weight are in percentages
    (
        "pure-pathing-s33-d33-t33",
        behaviors::pure_pathing_s33_d33_t33,
    ),
    (
        "pure-pathing-s10-d30-t60",
        behaviors::pure_pathing_s10_d30_t60,
    ),
    (
        "pure-pathing-s80-d10-t10",
        behaviors::pure_pathing_s80_d10_t10,
    ),
    (
        "pure-pathing-s10-d80-t10",
        behaviors::pure_pathing_s10_d80_t10,
    ),
    (
        "pure-pathing-s10-d10-t80",
        behaviors::pure_pathing_s10_d10_t80,
    ),
    (
        "pure-pathing-s20-d60-t20",
        behaviors::pure_pathing_s20_d60_t20,
    ),
    (
        "pure-pathing-s10-d45-t45",
        behaviors::pure_pathing_s10_d45_t45,
    ),
    (
        "pure-pathing-s0-d50-t50",
        behaviors::pure_pathing_s0_d50_t50,
    ),
    (
        "pure-pathing-s100-d0-t0",
        behaviors::pure_pathing_s100_d0_t0,
    ),
    (
        "pure-pathing-s0-d100-t0",
        behaviors::pure_pathing_s0_d100_t0,
    ),
    (
        "pure-pathing-s0-d0-t100",
        behaviors::pure_pathing_s0_d0_t100,
    ),
];

/// The range of the lidar sensor at which the robot moves away from an object
const LIDAR_OBSTACLE_RANGE: f32 = 1.0;

const GRADIENT_WEIGHT: f32 = 2.0;
const LIDAR_WEIGHT: f32 = 0.3;
const FORWARD_BIAS: f32 = 0.05;

const ANGLE_THRESHOLD: f32 = PI / 8.0;

const SEARCH_GRID_SCALE: f32 = 0.10;
const SEARCH_GRADIENT_RANGE: f32 = 5.0;
/// The threshold at which the robot will switch from exploring to pathing
const SEARCH_GRADIENT_EXPLORING_THRESHOLD: f32 = 0.1;

/// How often to update the search grid (multiplied on all changes to the cells)
const SEARCH_GRID_UPDATE_INTERVAL: f32 = 0.1;

const PROXIMITY_GRID_SCALE: f32 = 0.5;
const PROXIMITY_GRID_UPDATE_INTERVAL: f32 = 2.0;
const PROXIMITY_WEIGHT: f32 = 10.0;
const PROXIMITY_GRADIENT_RANGE: f32 = 10.0;
const PROXIMITY_MAX_LAYERS: usize = 3;

const COSTMAP_GRID_UPDATE_INTERVAL: f32 = 1.0;

const PATHING_ATEMPTS: u16 = 50;

/// How long to wait before retrying to path
const PATHING_FAIL_TIMEOUT_RANGE: Range<f32> = 0.1..30.0;

const ROBOT_SPACING: f32 = 2.0;

/// Min distance allowed from the center of the robot to an obstacle
const ROBOT_OBSTACLE_CLEARANCE: f32 = params::DIAMETER;

#[derive(Clone, Debug, Default, PartialEq)]
enum RobotMode {
    #[default]
    Exploring,
    Pathing,
    ProximityPathing,
}

#[derive(Clone, Default, Debug, PartialEq)]
enum CostmapCell {
    #[default]
    Unknown,
    Obstacle,
    DynamicObstacle,
    Searched,
}

impl CostmapCell {
    fn maybe_free(&self) -> bool {
        !self.is_obstacle()
    }

    fn is_obstacle(&self) -> bool {
        matches!(self, CostmapCell::Obstacle | CostmapCell::DynamicObstacle)
    }
}

impl From<CostmapCell> for f32 {
    fn from(cell: CostmapCell) -> Self {
        match cell {
            CostmapCell::Unknown => 1.0,
            CostmapCell::Obstacle => -3.0,
            CostmapCell::DynamicObstacle => -2.0,
            CostmapCell::Searched => -1.0,
        }
    }
}

type Costmap = ScaledGrid<CostmapCell>;

#[derive(Clone, Default)]
pub(crate) struct SearchRobot {
    /// The id of the robot
    id: RobotId,

    /// The position of the robot
    pos: Pos2,

    /// The angle of the robot
    angle: f32,

    /// The data from the camera. Angles and probability of objects.
    cam: CamData,

    /// The data from the lidar. Distance to objects.
    lidar: LidarData,

    /// The map navigating in
    map: Map,

    /// For sending/receiving messages
    postbox: Postbox,

    /// Grid containing probabilities of objects in the environment.
    search_grid: ScaledGrid<f32>,

    /// The time of the last search grid update
    last_search_grid_update: Duration,

    /// Grid containing position preferences for the robot based on
    /// the positions of other robots
    proximity_grid: ScaledGrid<f32>,

    /// The time of the last proximity grid update
    last_proximity_grid_update: Duration,

    /// Grid containing the costmap for path planning
    costmap_grid: ScaledGrid<CostmapCell>,

    /// The time of the last costmap grid update
    last_costmap_grid_update: Duration,

    /// Frontiers
    frontiers_grid: ScaledGrid<f32>,

    /// Frontier evaluation weights
    frontier_evaluation_weights: frontiers::FrontierEvaluationWeights,

    /// The goal of the path planner
    path_planner_goal: Option<Pos2>,

    /// The path planned by the robot
    path_planner_path: Vec<Pos2>,

    /// The path planned by the robot
    path_fails: u16,

    /// How long to wait before retrying to path (within `PATHING_FAIL_TIMEOUT_RANGE`)
    path_fail_timeout: f32,

    /// The time of the last planning failure
    path_fail_time: Option<Duration>,

    /// Other robots and their positions
    others: HashMap<RobotId, (Pos2, f32)>,

    /// Debug object and their names. Used for visualization.
    /// Set to `None` to disable debug visualization.
    debug_soup: DebugSoup,

    robot_mode: RobotMode,
}

impl Robot for SearchRobot {
    fn get_id(&self) -> &RobotId {
        &self.id
    }

    fn set_id(&mut self, id: RobotId) {
        self.id = id;
    }

    fn set_map(&mut self, map: Map) {
        let size = map.size();
        self.map = map;
        self.search_grid = ScaledGrid::new(size.x, size.y, SEARCH_GRID_SCALE);
        self.proximity_grid = ScaledGrid::new(size.x, size.y, PROXIMITY_GRID_SCALE);
        self.costmap_grid = ScaledGrid::new(size.x, size.y, COSTMAP_GRID_SCALE);
        self.frontiers_grid = ScaledGrid::new(size.x, size.y, COSTMAP_GRID_SCALE);
    }

    fn get_postbox(&self) -> &super::Postbox {
        &self.postbox
    }

    fn get_postbox_mut(&mut self) -> &mut super::Postbox {
        &mut self.postbox
    }

    fn get_debug_soup(&self) -> &super::DebugSoup {
        &self.debug_soup
    }

    fn get_debug_soup_mut(&mut self) -> &mut DebugSoup {
        &mut self.debug_soup
    }

    fn input_pose(&mut self, pose: RobotPose) {
        self.pos = pose.pos;
        self.angle = pose.angle;
    }

    fn input_cam(&mut self, cam: CamData) {
        self.cam = cam;
    }

    fn input_lidar(&mut self, lidar: LidarData) {
        self.lidar = lidar;
    }

    fn clone_box(&self) -> Box<dyn Robot> {
        Box::new(self.clone())
    }

    fn any(&self) -> &dyn std::any::Any {
        self
    }

    fn any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

impl SearchRobot {
    fn update_search_grid(&mut self, time: Duration) {
        // Only update the search grid every UPDATE_INTERVAL seconds
        if (time - self.last_search_grid_update).as_secs_f32() < SEARCH_GRID_UPDATE_INTERVAL {
            return;
        }
        self.last_search_grid_update = time;

        common::update_search_grid(
            &self.map,
            &mut self.search_grid,
            self.id,
            self.pos,
            self.angle,
            &mut self.postbox,
            &self.lidar,
            &self.cam,
            &mut self.others,
            SEARCH_GRID_UPDATE_INTERVAL,
        );
    }

    fn update_proximity_grid_naive(&mut self, time: Duration) {
        // Don't update the proximity grid too often
        if (time - self.last_proximity_grid_update).as_secs_f32() < PROXIMITY_GRID_UPDATE_INTERVAL {
            return;
        }
        self.last_proximity_grid_update = time;

        self.proximity_grid.fill(0.0);

        for (pos, _angle) in self.others.values() {
            let circle = Circle {
                center: *pos,
                radius: params::COMMUNICATION_RANGE,
            };

            for point in self.proximity_grid.iter_circle(&circle).collect::<Vec<_>>() {
                let d = (point - *pos).length();
                if d < ROBOT_SPACING {
                    continue;
                }
                if let Some(cell) = self.proximity_grid.get(point) {
                    self.proximity_grid.set(point, *cell + PROXIMITY_WEIGHT);
                }
            }
        }
    }

    fn update_proximity_grid(&mut self, time: Duration) {
        // Don't update the proximity grid too often
        if (time - self.last_proximity_grid_update).as_secs_f32() < PROXIMITY_GRID_UPDATE_INTERVAL {
            return;
        }
        self.last_proximity_grid_update = time;

        self.proximity_grid.fill(0.0);

        fn assign(
            id: RobotId,
            robots: &Vec<(&RobotId, &(Pos2, f32))>,
            assigned: &mut HashSet<RobotId>,
            network: &mut Vec<Pos2>,
            level: usize,
        ) {
            if assigned.contains(&id) {
                return;
            }

            if level > robots.len() {
                unreachable!("A robot should be assigned for each call to `assign`")
            }

            let (_, (pos, _)) = *robots.iter().find(|(i, _)| *i == &id).unwrap();
            network.push(*pos);
            assigned.insert(id);

            for (other_id, (other_pos, _)) in robots {
                if **other_id == id {
                    continue;
                }
                let d = (*pos - *other_pos).length();
                if d < COMMUNICATION_RANGE {
                    assign(**other_id, robots, assigned, network, level + 1);
                }
            }
        }

        let mut networks = vec![];
        let mut assigned = HashSet::default();

        let robots: Vec<_> = self.others.iter().collect();
        let ids = robots
            .iter()
            .map(|(id, _)| *id)
            .cloned()
            .collect::<Vec<_>>();

        loop {
            let Some(id) = ids.iter().find(|id| !assigned.contains(*id)) else {
                break;
            };
            let mut network = vec![];
            assign(*id, &robots, &mut assigned, &mut network, 0);
            networks.push(network);
        }

        let Some(largest_network) = networks.iter().max_by_key(|network| network.len()) else {
            return;
        };

        for pos in largest_network {
            let circle = Circle {
                center: *pos,
                radius: params::COMMUNICATION_RANGE,
            };

            for point in self.proximity_grid.iter_circle(&circle).collect::<Vec<_>>() {
                let d = (point - *pos).length();
                if d < ROBOT_SPACING {
                    continue;
                }
                if let Some(cell) = self.proximity_grid.get(point) {
                    let w = (*cell + PROXIMITY_WEIGHT)
                        .min(PROXIMITY_WEIGHT * PROXIMITY_MAX_LAYERS as f32);
                    self.proximity_grid.set(point, w);
                }
            }
        }
    }
}

impl SearchRobot {
    fn search_gradient(&mut self) -> Vec2 {
        let (g, cells) = common::gradient(
            self.pos,
            self.angle,
            SEARCH_GRADIENT_RANGE,
            params::DIAMETER * 2.0,
            self.lidar.clone(),
            &self.search_grid,
        );
        let g = g * GRADIENT_WEIGHT;

        self.debug("Gradient", "Search Cells", DebugItem::NumberPoints(cells));
        self.debug("Gradient", "Search Gradient", DebugItem::Vector(g));

        // let g_len = g.length();
        // if g_len < SEARCH_GRADIENT_EXPLORING_THRESHOLD {
        //     // println!(
        //     //     "[{}] Switching to pathing: gradient: {}",
        //     //     self.id.as_u32(),
        //     //     g_len
        //     // );
        //     self.robot_mode = RobotMode::Pathing;
        // }
        g
    }

    fn proximity_gradient(&mut self) -> Vec2 {
        let (g, _) = common::gradient(
            self.pos,
            self.angle,
            PROXIMITY_GRADIENT_RANGE,
            params::DIAMETER * 2.0,
            self.lidar.clone(),
            &self.proximity_grid,
        );
        let g = g * GRADIENT_WEIGHT;

        self.debug("Gradient", "Proximity Gradient", DebugItem::Vector(g));
        self.debug(
            "Gradient",
            "Proximity Gradient Range",
            DebugItem::Radius(PROXIMITY_GRADIENT_RANGE),
        );
        self.debug(
            "",
            "Communication Range",
            DebugItem::Radius(params::COMMUNICATION_RANGE),
        );
        g
    }

    /// Calculate the lidar contribution to the control
    fn lidar(&mut self) -> Vec2 {
        let mut lidar_contribution = Vec2::ZERO;
        {
            let mut total_weight: f32 = 0.0;
            for point in self.lidar.points() {
                let distance = point.distance.clamp(0.0, LIDAR_OBSTACLE_RANGE);
                let weight = -(1.0 - distance / LIDAR_OBSTACLE_RANGE).powi(2);
                lidar_contribution += Vec2::angled(point.angle + self.angle) * weight;
            }

            if total_weight == 0.0 {
                total_weight = 1.0;
            }
            lidar_contribution /= total_weight.abs();

            lidar_contribution *= LIDAR_WEIGHT;

            self.debug(
                "Sensors",
                "Lidar Obstacle Range",
                DebugItem::Radius(LIDAR_OBSTACLE_RANGE),
            );

            self.debug(
                "Sensors",
                "Lidar Contribution",
                DebugItem::Vector(lidar_contribution),
            );
        }
        lidar_contribution
    }

    fn control_towards(&self, target: Vec2) -> Control {
        if target == Vec2::ZERO {
            return Control::default();
        }
        let angle_error = normalize_angle(self.angle - target.angle());
        let t = (angle_error * angle_error / ANGLE_THRESHOLD).clamp(0.0, 1.0);
        let speed = 1.0 - t;
        let steer = normalize_angle(target.angle() - self.angle);

        Control { speed, steer }
    }

    fn visualize(&mut self) {
        let soup = &mut self.debug_soup;

        let lidar = &self.lidar;
        debug_soup::common_routines::show_lidar(soup, lidar);

        debug_soup::common_routines::show_cam_range(soup, lidar);

        soup.add(
            "",
            "Other Positions",
            DebugItem::VectorField(
                self.others
                    .values()
                    .map(|(pos, angle)| (*pos, Vec2::angled(*angle) * params::DIAMETER / 2.0))
                    .collect(),
            ),
        );

        soup.add(
            "Grids",
            "Search Grid",
            DebugItem::Grid(self.search_grid.clone()),
        );

        soup.add(
            "Grids",
            "Proximity Grid",
            DebugItem::Grid(self.proximity_grid.clone()),
        );

        soup.add(
            "Grids",
            "Costmap Grid",
            DebugItem::Grid(self.costmap_grid.clone().transform(f32::from)),
        );

        soup.add(
            "Grids",
            "Frontiers",
            DebugItem::Grid(self.frontiers_grid.clone()),
        );

        soup.add("Grids", "Map", DebugItem::Map(self.map.clone()));

        let mode = match &self.robot_mode {
            RobotMode::Exploring => 0,
            RobotMode::Pathing => 1,
            RobotMode::ProximityPathing => 2,
        };

        soup.add("", "mode", DebugItem::Int(mode));

        if matches!(
            self.robot_mode,
            RobotMode::Pathing | RobotMode::ProximityPathing
        ) {
            self.show_path();
            let goal = self.path_planner_goal.unwrap_or(Pos2 { x: 0.0, y: 0.0 });
            self.get_debug_soup_mut()
                .add("Planner", "Goal", DebugItem::Point(goal));
        }
    }
}

/// Function related to path planning
impl SearchRobot {
    fn update_costmap_grid(&mut self, time: Duration) {
        // Don't update the costmap grid too often
        if (time - self.last_costmap_grid_update).as_secs_f32() < COSTMAP_GRID_UPDATE_INTERVAL {
            return;
        }
        self.last_costmap_grid_update = time;

        self.costmap_grid = costmap::make_costmap_grid(
            self.pos,
            self.angle,
            &self.map,
            &self.search_grid,
            &self.lidar,
        );
    }

    fn set_path_timeout(&mut self, time: Duration) {
        self.path_planner_goal = None;
        self.path_planner_path = vec![];
        self.robot_mode = RobotMode::Exploring;

        self.path_fail_time = Some(time);
        self.path_fail_timeout = (self.path_fail_timeout * 2.0).clamp(
            PATHING_FAIL_TIMEOUT_RANGE.start,
            PATHING_FAIL_TIMEOUT_RANGE.end,
        );
        println!(
            "[{}] Pathing failed. Waiting for {} seconds",
            self.id.as_u32(),
            self.path_fail_timeout
        );
    }

    fn set_path_success(&mut self) {
        self.path_fails = 0;
        self.path_fail_timeout = PATHING_FAIL_TIMEOUT_RANGE.start;
    }

    /// Returns target vector or `None` if no path is found
    fn path_planning(&mut self, time: Duration) -> Option<Vec2> {
        // Don't path if we failed recently
        if let Some(path_fail_time) = self.path_fail_time {
            if (time - path_fail_time).as_secs_f32() < self.path_fail_timeout {
                return None;
            }
        }

        if self.path_fails > PATHING_ATEMPTS {
            self.path_fails = 0;
            self.set_path_timeout(time);
            println!("[{}] Pathing failed too many times", self.id.as_u32());
            return None;
        }

        // Logic to find goal or change to exploring mode when reached
        if let Some(goal) = self.path_planner_goal {
            let diff = goal - self.pos;
            // Have we reached the goal?
            if diff.length() < PATH_PLANNER_DISTANCE_TOLERANCE {
                self.path_planner_goal = None;
                self.path_planner_path = vec![];
                self.robot_mode = RobotMode::Exploring;
                return None;
            }
        } else {
            // Set a goal using frontiers
            let mut frontiers = frontiers::find_frontiers(self.pos, &self.costmap_grid);
            self.show_frontiers(&frontiers);
            if frontiers.is_empty() {
                for (pos, _angle) in self.others.values() {
                    let robot_frontiers = frontiers::find_frontiers(*pos, &self.costmap_grid);
                    if !robot_frontiers.is_empty() {
                        frontiers = robot_frontiers;
                        break;
                    }
                }
            }

            // Abort if we still have no frontiers
            if frontiers.is_empty() {
                self.path_fails += 1;
                return None;
            }

            match frontiers::evaluate_frontiers(
                self.pos,
                self.angle,
                self.frontier_evaluation_weights,
                frontiers,
                &self.costmap_grid,
            ) {
                Some(goal) => self.path_planner_goal = Some(goal),
                None => {
                    // Should only happen in the beginning
                    self.set_path_timeout(time);
                    return None;
                }
            }
        }

        self.set_path_success();

        // Don't stop pathing if there are no other robots in map
        if !self.others.is_empty() && self.robot_mode != RobotMode::ProximityPathing {
            match self.proximity_grid.get(self.pos) {
                // 0.0 means we are outside the proximity grid
                Some(cell) if *cell == 0.0 => {
                    let mut masked_costmap = self.costmap_grid.clone();
                    // Make the grid to find a new goal within the proximity grid

                    masked_costmap.mask(|pos| {
                        if let Some(proximity_cell) = self.proximity_grid.get(pos) {
                            // Don't overwrite obstacles
                            if let Some(costmap_cell) = self.costmap_grid.get(pos) {
                                costmap_cell.is_obstacle() || *proximity_cell != 0.0
                            } else {
                                // Keep the cell if it is in the proximity grid
                                *proximity_cell != 0.0
                            }
                        } else {
                            false
                        }
                    });

                    // Set a goal from current position
                    let mut frontiers = frontiers::find_frontiers(self.pos, &masked_costmap);

                    // If no frontiers, try to find a frontier from other robots positions
                    if frontiers.is_empty() {
                        for (pos, _angle) in self.others.values() {
                            let robot_frontiers = frontiers::find_frontiers(*pos, &masked_costmap);
                            if !robot_frontiers.is_empty() {
                                frontiers = robot_frontiers;
                                break;
                            }
                        }
                    }

                    // If still no frontiers, use the current position with full costmap
                    if frontiers.is_empty() {
                        // As backup
                        frontiers = frontiers::find_frontiers(self.pos, &self.costmap_grid);
                    }

                    self.show_frontiers(&frontiers);

                    self.get_debug_soup_mut().add(
                        "Planner",
                        "Masked Costmap",
                        DebugItem::Grid(masked_costmap.clone().transform(f32::from)),
                    );

                    // self.get_debug_soup_mut().add(
                    //     "Planner",
                    //     "Masked frontiers",
                    //     DebugItem::Grid(masked_costmap.clone()),
                    // );

                    // let frontier_len = frontiers.len();

                    match frontiers::evaluate_frontiers(
                        self.pos,
                        self.angle,
                        self.frontier_evaluation_weights,
                        frontiers,
                        &masked_costmap,
                    ) {
                        Some(goal) => {
                            // println!("[{}] Set prox goal", self.id.0);
                            self.get_debug_soup_mut().add(
                                "Planner",
                                "Goal",
                                DebugItem::Point(goal),
                            );

                            self.robot_mode = RobotMode::ProximityPathing;
                            self.path_planner_goal = Some(goal);
                        }
                        None => {
                            // println!("[{}] No prox goal. N frontiers={}", self.id.0, frontier_len,);
                            // Should only happen in the beginning
                            self.robot_mode = RobotMode::Exploring;
                            return None;
                        }
                    }
                }
                None => {
                    // Robot not in proximity grid
                }
                _ => {}
            }
        }

        // Follow path or reevaluate path or goal?
        if self.path_planner_path.is_empty() {
            if let Some(path) = pathing::find_path(
                self.pos,
                self.path_planner_goal
                    .expect("Path planner goal should be set"),
                &self.costmap_grid,
            ) {
                self.path_planner_path = path;
                // Start following the path next time
                None
            } else {
                Some(self.lidar())
            }
        } else {
            // Update the path to only contain the points we haven't reached
            if (self.pos - self.path_planner_path[0]).length() < PATH_PLANNER_DISTANCE_TOLERANCE {
                self.path_planner_path.remove(0);
            }

            // Follow the path
            self.follow_path().ok()
        }
    }

    fn follow_path(&mut self) -> Result<Vec2, ()> {
        if !self.path_planner_path.is_empty() {
            // Check if all cells in the line to the goal are free
            let line = Line {
                start: self.pos,
                end: self.path_planner_path[0],
            };

            let target = (self.path_planner_path[0] - self.pos).normalized();

            match costmap::validate_thick_line(line, ROBOT_OBSTACLE_CLEARANCE, &self.costmap_grid) {
                true => {
                    self.path_fails = 0;
                    Ok(target)
                }
                false => {
                    // Track failures to set new goal or path
                    self.path_fails += 1;
                    Ok((target + self.lidar()).normalized())
                }
            }
        } else {
            println!("[{}] Path is empty", self.id.0);
            Err(())
        }
    }

    fn show_path(&mut self) {
        if self.path_planner_goal.is_some() {
            let soup = &mut self.debug_soup;
            let mut path = Vec::with_capacity(self.path_planner_path.len() + 1);
            path.push(self.pos);
            path.extend(self.path_planner_path.iter().cloned());
            soup.add("Planner", "Goal Path", DebugItem::GlobalLine(path));
        }
    }

    fn show_frontiers(&mut self, frontiers: &HashSet<(usize, usize)>) {
        self.frontiers_grid.fill(0.0);
        for (x, y) in frontiers {
            self.frontiers_grid.grid_mut().set(*x, *y, -10.0);
        }
        let robot_pos = {
            let tmp = self.frontiers_grid.pos_to_grid(self.pos);
            (tmp.x as usize, tmp.y as usize)
        };
        let mut frontier_regions =
            frontiers::make_frontier_regions(frontiers.clone(), &self.costmap_grid);

        frontier_regions.sort_by_key(|v1| v1.len());

        let frontier_regions_index: Vec<(Pos2, f32)> = frontier_regions
            .iter()
            .rev() // Smallest idx for biggest regions
            .enumerate()
            .flat_map(|(idx, region)| {
                region
                    .iter()
                    .map(|(x, y)| {
                        let idx = idx as f32;
                        let pos = self
                            .frontiers_grid
                            .grid_to_pos(Pos2::new(*x as f32, *y as f32));
                        (pos, idx)
                    })
                    .collect::<Vec<_>>()
            })
            .collect();

        let best = frontiers::evaluate_frontier_regions(
            robot_pos,
            self.angle,
            self.frontier_evaluation_weights,
            frontier_regions,
            &self.costmap_grid,
        );

        self.get_debug_soup_mut().add(
            "Planner",
            "Frontier Regions Index",
            DebugItem::NumberPoints(frontier_regions_index),
        );

        if let Some(best) = best {
            let pos = Pos2::new(best.0 as f32, best.1 as f32);
            let pos = self.frontiers_grid.grid_to_pos(pos);
            self.get_debug_soup_mut()
                .add("Planner", "Frontier Target", DebugItem::Point(pos));
        }
    }
}

fn search(
    robot: &mut RobotRef,
    time: Duration,
    update: fn(&mut SearchRobot, time: Duration),
    target: fn(&mut SearchRobot, time: Duration) -> Vec2,
) -> BehaviorOutput {
    let robot = cast_robot::<SearchRobot>(robot);

    // Debug visualization
    robot.visualize();

    update(robot, time);

    let target = target(robot, time);

    robot.debug("", "Target", DebugItem::Vector(target));

    robot.postbox.clean();

    let msgs = robot.postbox.empty();
    (robot.control_towards(target), msgs)
}

mod behaviors {
    use super::*;

    pub fn full(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                if robot.robot_mode == RobotMode::Pathing {
                    robot.update_costmap_grid(time);
                }
            },
            |robot, time| match robot.robot_mode {
                RobotMode::Exploring => {
                    let mut target = Vec2::angled(robot.angle) * FORWARD_BIAS;
                    let search_gradient = robot.search_gradient();
                    let proximity_gradient = robot.proximity_gradient();
                    let lidar = robot.lidar();
                    target += search_gradient + proximity_gradient + lidar;
                    if search_gradient.length() < SEARCH_GRADIENT_EXPLORING_THRESHOLD {
                        robot.robot_mode = RobotMode::Pathing;
                    }
                    target
                }
                RobotMode::Pathing | RobotMode::ProximityPathing => {
                    robot.path_planning(time).unwrap_or(Vec2::ZERO)
                }
            },
        )
    }

    pub fn naive_proximity(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid_naive(time);
            },
            |robot, _time| {
                let mut target = Vec2::angled(robot.angle) * FORWARD_BIAS;
                target += robot.search_gradient();
                target += robot.proximity_gradient();
                target
            },
        )
    }

    pub fn no_proximity(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
            },
            |robot, _time| {
                let mut target = Vec2::angled(robot.angle) * FORWARD_BIAS;
                target += robot.search_gradient();
                target
            },
        )
    }

    pub fn no_pathing(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
            },
            |robot, _time| {
                let mut target = Vec2::angled(robot.angle) * FORWARD_BIAS;
                target += robot.search_gradient();
                target += robot.proximity_gradient();
                target
            },
        )
    }

    pub fn pure_pathing(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.robot_mode = RobotMode::Pathing;
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
            },
            |robot, time| robot.path_planning(time).unwrap_or(Vec2::ZERO),
        )
    }

    // Experimental
    pub fn pure_pathing_s33_d33_t33(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
                // Make sure we stay in pathing mode always
                robot.robot_mode = RobotMode::Pathing;
            },
            |robot, time| {
                let target = robot.path_planning(time);
                robot.frontier_evaluation_weights = frontiers::FrontierEvaluationWeights {
                    frontier_region_size: 0.3333,
                    frontier_distance: 0.3333,
                    frontier_turn: 0.3333,
                };
                target.unwrap_or(Vec2::ZERO)
            },
        )
    }
    pub fn pure_pathing_s10_d30_t60(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
                // Make sure we stay in pathing mode always
                robot.robot_mode = RobotMode::Pathing;
            },
            |robot, time| {
                let target = robot.path_planning(time);
                robot.frontier_evaluation_weights = frontiers::FrontierEvaluationWeights {
                    frontier_region_size: 0.1,
                    frontier_distance: 0.3,
                    frontier_turn: 0.6,
                };
                target.unwrap_or(Vec2::ZERO)
            },
        )
    }
    pub fn pure_pathing_s80_d10_t10(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
                // Make sure we stay in pathing mode always
                robot.robot_mode = RobotMode::Pathing;
            },
            |robot, time| {
                let target = robot.path_planning(time);
                robot.frontier_evaluation_weights = frontiers::FrontierEvaluationWeights {
                    frontier_region_size: 0.8,
                    frontier_distance: 0.1,
                    frontier_turn: 0.1,
                };
                target.unwrap_or(Vec2::ZERO)
            },
        )
    }
    pub fn pure_pathing_s10_d80_t10(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
                // Make sure we stay in pathing mode always
                robot.robot_mode = RobotMode::Pathing;
            },
            |robot, time| {
                let target = robot.path_planning(time);
                robot.frontier_evaluation_weights = frontiers::FrontierEvaluationWeights {
                    frontier_region_size: 0.1,
                    frontier_distance: 0.8,
                    frontier_turn: 0.1,
                };
                target.unwrap_or(Vec2::ZERO)
            },
        )
    }
    pub fn pure_pathing_s10_d10_t80(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
                // Make sure we stay in pathing mode always
                robot.robot_mode = RobotMode::Pathing;
            },
            |robot, time| {
                let target = robot.path_planning(time);
                robot.frontier_evaluation_weights = frontiers::FrontierEvaluationWeights {
                    frontier_region_size: 0.1,
                    frontier_distance: 0.1,
                    frontier_turn: 0.8,
                };
                target.unwrap_or(Vec2::ZERO)
            },
        )
    }
    pub fn pure_pathing_s20_d60_t20(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
                // Make sure we stay in pathing mode always
                robot.robot_mode = RobotMode::Pathing;
            },
            |robot, time| {
                let target = robot.path_planning(time);
                robot.frontier_evaluation_weights = frontiers::FrontierEvaluationWeights {
                    frontier_region_size: 0.2,
                    frontier_distance: 0.6,
                    frontier_turn: 0.2,
                };
                target.unwrap_or(Vec2::ZERO)
            },
        )
    }
    pub fn pure_pathing_s10_d45_t45(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
                // Make sure we stay in pathing mode always
                robot.robot_mode = RobotMode::Pathing;
            },
            |robot, time| {
                let target = robot.path_planning(time);
                robot.frontier_evaluation_weights = frontiers::FrontierEvaluationWeights {
                    frontier_region_size: 0.1,
                    frontier_distance: 0.45,
                    frontier_turn: 0.45,
                };
                target.unwrap_or(Vec2::ZERO)
            },
        )
    }
    pub fn pure_pathing_s0_d50_t50(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
                // Make sure we stay in pathing mode always
                robot.robot_mode = RobotMode::Pathing;
            },
            |robot, time| {
                let target = robot.path_planning(time);
                robot.frontier_evaluation_weights = frontiers::FrontierEvaluationWeights {
                    frontier_region_size: 0.0,
                    frontier_distance: 0.5,
                    frontier_turn: 0.5,
                };
                target.unwrap_or(Vec2::ZERO)
            },
        )
    }
    pub fn pure_pathing_s100_d0_t0(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
                // Make sure we stay in pathing mode always
                robot.robot_mode = RobotMode::Pathing;
            },
            |robot, time| {
                let target = robot.path_planning(time);
                robot.frontier_evaluation_weights = frontiers::FrontierEvaluationWeights {
                    frontier_region_size: 1.0,
                    frontier_distance: 0.0,
                    frontier_turn: 0.0,
                };
                target.unwrap_or(Vec2::ZERO)
            },
        )
    }
    pub fn pure_pathing_s0_d100_t0(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
                // Make sure we stay in pathing mode always
                robot.robot_mode = RobotMode::Pathing;
            },
            |robot, time| {
                let target = robot.path_planning(time);
                robot.frontier_evaluation_weights = frontiers::FrontierEvaluationWeights {
                    frontier_region_size: 0.0,
                    frontier_distance: 1.0,
                    frontier_turn: 0.0,
                };
                target.unwrap_or(Vec2::ZERO)
            },
        )
    }
    pub fn pure_pathing_s0_d0_t100(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
        search(
            robot,
            time,
            |robot, time| {
                robot.update_search_grid(time);
                robot.update_proximity_grid(time);
                robot.update_costmap_grid(time);
                // Make sure we stay in pathing mode always
                robot.robot_mode = RobotMode::Pathing;
            },
            |robot, time| {
                let target = robot.path_planning(time);
                robot.frontier_evaluation_weights = frontiers::FrontierEvaluationWeights {
                    frontier_region_size: 0.0,
                    frontier_distance: 0.0,
                    frontier_turn: 1.0,
                };
                target.unwrap_or(Vec2::ZERO)
            },
        )
    }
}
