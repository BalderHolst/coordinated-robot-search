//! This module contains robot `search` behavior.

use std::{
    collections::{HashMap, HashSet},
    f32::consts::PI,
    time::Duration,
};

use emath::{Pos2, Vec2};

use crate::{params::COMMUNICATION_RANGE, LidarData};

use super::{
    cast_robot, common, debug,
    params::{self, LIDAR_RANGE},
    scaled_grid::ScaledGrid,
    shapes::{Circle, Line},
    utils::normalize_angle,
    BehaviorFn, BehaviorOutput, CamData, Control, DebugSoup, DebugType, LidarPoint, MapCell,
    Postbox, Robot, RobotId, RobotPose, RobotRef,
};

pub const MENU: &[(&str, BehaviorFn)] = &[
    ("full", behaviors::full),
    ("naive-proximity", behaviors::naive_proximity),
    ("no-proximity", behaviors::no_proximity),
    ("no-pathing", behaviors::no_pathing),
];

/// The range of the lidar sensor at which the robot moves away from an object
const LIDAR_OBSTACLE_RANGE: f32 = 1.0;

const GRADIENT_WEIGHT: f32 = 2.0;
const LIDAR_WEIGHT: f32 = 0.3;
const FORWARD_BIAS: f32 = 0.05;

const ANGLE_THRESHOLD: f32 = PI / 4.0;

const SEARCH_GRID_SCALE: f32 = 0.20;
const SEARCH_GRADIENT_RANGE: f32 = 5.0;
/// The threshold at which the robot will switch from exploring to pathing
const SEARCH_GRADIENT_EXPLORING_THRESHOLD: f32 = 0.02;

/// How often to update the search grid (multiplied on all changes to the cells)
const SEARCH_GRID_UPDATE_INTERVAL: f32 = 0.1;

const PROXIMITY_GRID_SCALE: f32 = 1.00;
const PROXIMITY_GRID_UPDATE_INTERVAL: f32 = 2.0;
const PROXIMITY_WEIGHT: f32 = 10.0;
const PROXIMITY_GRADIENT_RANGE: f32 = 10.0;
const PROXIMITY_MAX_LAYERS: usize = 3;

const COSTMAP_GRID_SCALE: f32 = 0.5;
const COSTMAP_GRID_UPDATE_INTERVAL: f32 = 1.0;

const COSTMAP_OCCUPIED: f32 = -3.0;
const COSTMAP_DYNAMIC_OBSTACLE: f32 = -1.0;
const COSTMAP_SEARCHED: f32 = -1.0;
const COSTMAP_UNKNOWN: f32 = 1.0;

const PATH_PLANNER_GOAL_TOLERANCE: f32 = 0.5;

const ROBOT_SPACING: f32 = 4.0;

#[derive(Clone, Debug, Default, PartialEq)]
pub enum RobotMode {
    #[default]
    Exploring,
    Pathing,
}

#[derive(Clone, Default)]
pub struct SearchRobot {
    /// The id of the robot
    pub id: RobotId,

    /// The position of the robot
    pub pos: Pos2,

    /// The velocity of the robot
    pub vel: f32,

    /// The angle of the robot
    pub angle: f32,

    /// The angular velocity of the robot
    pub avel: f32,

    /// The data from the camera. Angles and probability of objects.
    pub cam: CamData,

    /// The data from the lidar. Distance to objects.
    pub lidar: LidarData,

    /// The map navigating in
    pub map: ScaledGrid<MapCell>,

    /// For sending/receiving messages
    pub postbox: Postbox,

    /// Grid containing probabilities of objects in the environment.
    pub search_grid: ScaledGrid<f32>,

    /// The time of the last search grid update
    pub last_search_grid_update: Duration,

    /// Grid containing position preferences for the robot based on
    /// the positions of other robots
    pub proximity_grid: ScaledGrid<f32>,

    /// The time of the last proximity grid update
    pub last_proximity_grid_update: Duration,

    /// Grid containing the costmap for path planning
    pub costmap_grid: ScaledGrid<f32>,

    /// The time of the last costmap grid update
    pub last_costmap_grid_update: Duration,

    /// The goal of the path planner
    pub path_planner_goal: Option<Pos2>,

    /// The path planned by the robot
    pub path_planner_path: Vec<Pos2>,

    /// Other robots and their positions
    pub others: HashMap<RobotId, (Pos2, f32)>,

    /// Debug object and their names. Used for visualization.
    /// Set to `None` to disable debug visualization.
    pub debug_soup: DebugSoup,

    pub robot_mode: RobotMode,
}

impl Robot for SearchRobot {
    fn get_id(&self) -> &RobotId {
        &self.id
    }

    fn set_id(&mut self, id: RobotId) {
        self.id = id;
    }

    fn set_world(&mut self, world: ScaledGrid<MapCell>) {
        let size = world.size();
        self.map = world;
        self.search_grid = ScaledGrid::new(size.x, size.y, SEARCH_GRID_SCALE);
        self.proximity_grid = ScaledGrid::new(size.x, size.y, PROXIMITY_GRID_SCALE);
        self.costmap_grid = ScaledGrid::new(size.x, size.y, COSTMAP_GRID_SCALE);
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
        ) {
            if assigned.contains(&id) {
                return;
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
                    assign(**other_id, robots, assigned, network);
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
            assign(*id, &robots, &mut assigned, &mut network);
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
        let (g, cells) = gradient(
            self.pos,
            self.angle,
            SEARCH_GRADIENT_RANGE,
            params::DIAMETER * 2.0,
            self.lidar.clone(),
            &self.search_grid,
        );
        self.debug("Gradient", "Search Cells", DebugType::NumberPoints(cells));
        self.debug("Gradient", "Search Gradient", DebugType::Vector(g));

        let g_len = g.length();
        if g_len < SEARCH_GRADIENT_EXPLORING_THRESHOLD {
            println!("Switching to pathing: gradient: {}", g_len);
            self.robot_mode = RobotMode::Pathing;
        }
        g
    }

    fn proximity_gradient(&mut self) -> Vec2 {
        let (g, _) = gradient(
            self.pos,
            self.angle,
            PROXIMITY_GRADIENT_RANGE,
            params::DIAMETER * 2.0,
            self.lidar.clone(),
            &self.proximity_grid,
        );
        self.debug("Gradient", "Proximity Gradient", DebugType::Vector(g));
        self.debug(
            "Gradient",
            "Proximity Gradient Range",
            DebugType::Radius(PROXIMITY_GRADIENT_RANGE),
        );
        self.debug(
            "",
            "Communication Range",
            DebugType::Radius(params::COMMUNICATION_RANGE),
        );
        g
    }

    /// Calculate the lidar contribution to the control
    fn lidar(&mut self) -> Vec2 {
        let mut lidar_contribution = Vec2::ZERO;
        {
            let mut total_weight: f32 = 0.0;
            let LidarData(points) = self.lidar.clone();

            for point in points {
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
                DebugType::Radius(LIDAR_OBSTACLE_RANGE),
            );

            self.debug(
                "Sensors",
                "Lidar Contribution",
                DebugType::Vector(lidar_contribution),
            );
        }
        lidar_contribution
    }

    pub fn control_towards(&self, target: Option<Vec2>) -> Control {
        if let Some(target) = target {
            let angle_error = normalize_angle(self.angle - target.angle());
            let t = (angle_error * angle_error / ANGLE_THRESHOLD).clamp(0.0, 1.0);
            let speed = 1.0 - t;
            let steer = normalize_angle(target.angle() - self.angle);

            Control { speed, steer }
        } else {
            Control {
                speed: 0.0,
                steer: 0.0,
            }
        }
    }

    fn visualize(&mut self) {
        let soup = &mut self.debug_soup;

        let lidar = &self.lidar;
        debug::common_routines::show_lidar(soup, lidar);

        debug::common_routines::show_cam_range(soup, lidar);

        soup.add(
            "",
            "Other Positions",
            DebugType::VectorField(
                self.others
                    .values()
                    .map(|(pos, angle)| (*pos, Vec2::angled(*angle) * params::DIAMETER / 2.0))
                    .collect(),
            ),
        );

        soup.add(
            "Grids",
            "Search Grid",
            DebugType::Grid(self.search_grid.clone()),
        );

        soup.add(
            "Grids",
            "Proximity Grid",
            DebugType::Grid(self.proximity_grid.clone()),
        );

        soup.add(
            "Grids",
            "Costmap Grid",
            DebugType::Grid(self.costmap_grid.clone()),
        );

        // if self.robot_mode == RobotMode::Pathing {
        self.show_path();
        // }
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

        self.costmap_grid.fill(0.0);

        self.search_grid.iter().for_each(|(x, y, &cell)| {
            // Negative if don't want to go there, positive if want to go there
            // Non-zero cells are explored therefore we won't go there
            let cell = if cell != 0.0 {
                COSTMAP_SEARCHED
            } else {
                COSTMAP_UNKNOWN
            };
            self.costmap_grid.set(Pos2 { x, y }, cell);
        });

        // TODO: Update costmap from obstacles (but botbrain doesn't have the map?), costmap from only lidar then??
        self.lidar
            .clone()
            .points()
            .filter_map(|&LidarPoint { angle, distance }| {
                // Make small circle if distance is under max distance
                if distance < LIDAR_RANGE {
                    let point = self.pos + Vec2::angled(angle + self.angle) * distance;
                    Some(Circle {
                        center: point,
                        radius: 1.0, // The circle as made where the lidar hits
                    })
                } else {
                    None
                }
            })
            .for_each(|circle| {
                self.costmap_grid
                    // Negative if don't want to go there, positive if want to go there
                    .set_circle(circle.center, circle.radius, COSTMAP_DYNAMIC_OBSTACLE);
            });
    }

    fn path_planning(&mut self) -> Option<Vec2> {
        // Logic to find goal or change to exploring mode when reached
        if let Some(goal) = self.path_planner_goal {
            let diff = goal - self.pos;
            if diff.length() < PATH_PLANNER_GOAL_TOLERANCE {
                self.path_planner_goal = None;
                println!("Goal reached, switching to exploring mode");
                self.robot_mode = RobotMode::Exploring;
                return None;
            }
        } else {
            // Set a goal
            let goal = self.find_area_of_interest();
            self.path_planner_goal = Some(goal);
        }

        // Don't stop pathing if there are no other robots in world
        if !self.others.is_empty() {
            // Only limit robot movement if there are other robots in the proximity grid
            match self.proximity_grid.get(self.pos) {
                Some(cell) if *cell == 0.0 => {
                    // If we are not in the proximity grid, we can't move towards the goal
                    return None;
                }
                None => {
                    println!("Robot position not in proximity grid: {:?}", self.pos);
                }
                _ => {}
            }
        }

        // Follow path or reevaluate path or goal?
        if self.path_planner_path.is_empty() && self.find_path().is_err() {
            // TODO: Determine best way to proceed
            // New goal, new path or switch mode?
            println!("What: {:?}", self.robot_mode);
            // Mock vector for now
            Some(Vec2 { x: 0.0, y: 0.0 })
        } else {
            // Update the path to only contain the points we haven't reached
            if (self.pos - self.path_planner_path[0]).length() < PATH_PLANNER_GOAL_TOLERANCE {
                self.path_planner_path.remove(0);
            }

            // Follow the path
            if let Ok(control_vec) = self.follow_path() {
                // println!("Control vec: {}", control_vec);
                Some(control_vec)
            } else {
                // TODO: Determine best way to proceed
                // New goal, new path or switch mode?
                None
            }
        }
    }

    fn find_area_of_interest(&mut self) -> Pos2 {
        // TODO: Find a good way to choose the goal
        Pos2::new(25.0, -20.0)
    }

    fn follow_path(&self) -> Result<Vec2, ()> {
        if !self.path_planner_path.is_empty() {
            // Check if all cells in the line to the goal are free
            let line = Line {
                start: self.pos,
                end: self.path_planner_path[0],
            };

            let is_path_clear = self.costmap_grid.iter_line(&line).all(|pos| {
                if let Some(&cell) = self.costmap_grid.get(pos) {
                    cell != COSTMAP_OCCUPIED && cell != COSTMAP_DYNAMIC_OBSTACLE
                } else {
                    false
                }
            });

            if is_path_clear {
                Ok((self.path_planner_path[0] - self.pos).normalized())
            } else {
                Err(())
            }
        } else {
            Err(())
        }
    }

    fn find_straight_path(&self) -> Result<Vec<Pos2>, ()> {
        if let Some(goal) = self.path_planner_goal {
            let line = Line {
                start: self.pos,
                end: goal,
            };
            // Check if all cells in the line to the goal are free
            let res = self.costmap_grid.iter_line(&line).all(|pos| {
                if let Some(&cell) = self.costmap_grid.get(pos) {
                    // FIX: Remove COSTMAP_DYNAMIC_OBSTACLE when we have a map and not only lidar
                    cell != COSTMAP_OCCUPIED && cell != COSTMAP_DYNAMIC_OBSTACLE
                } else {
                    false
                }
            });

            if res {
                Ok(vec![self.pos, goal])
            } else {
                Err(())
            }
        } else {
            Err(())
        }
    }

    fn find_rrt_path(&self) -> Result<Vec<Pos2>, ()> {
        if let Some(goal) = self.path_planner_goal {
            // TODO: Use rrt to find path to goal
            Ok(vec![
                self.pos,
                Pos2::new(25.0, 25.0),
                Pos2::new(-25.0, 25.0),
                Pos2::new(-25.0, -25.0),
                Pos2::new(25.0, -25.0),
                goal,
            ])
        } else {
            Err(())
        }
    }

    fn find_path(&mut self) -> Result<(), ()> {
        self.path_planner_path = self.find_rrt_path().unwrap();
        Ok(())
        // if let Ok(path) = self.find_straight_path() {
        //     self.path_planner_path = path;
        //     Ok(())
        // } else if let Ok(path) = self.find_rrt_path() {
        //     self.path_planner_path = path;
        //     Ok(())
        // } else {
        //     Err(())
        // }
    }

    fn show_path(&mut self) {
        if self.path_planner_goal.is_some() {
            let soup = &mut self.debug_soup;
            let mut path = Vec::with_capacity(self.path_planner_path.len() + 1);
            path.push(self.pos);
            path.extend(self.path_planner_path.iter().cloned());
            soup.add("Planner", "Global Path", DebugType::GlobalLine(path));
        }
    }
}

/// Calculate the gradient of the heat map around the robot
fn gradient(
    pos: Pos2,
    angle: f32,
    range: f32,
    center_range: f32,
    lidar: LidarData,
    grid: &ScaledGrid<f32>,
) -> (Vec2, Vec<(Pos2, f32)>) {
    let mut total_heat = 0.0;
    let mut robot_points: usize = 0;
    for pos in grid.iter_circle(
        &(Circle {
            center: pos,
            radius: center_range,
        }),
    ) {
        let cell = grid.get(pos);
        total_heat += cell.unwrap_or(&0.0);
        robot_points += 1;
    }

    let robot_heat = total_heat / robot_points as f32;

    let mut cells = vec![];
    let mut gradient = Vec2::ZERO;
    {
        for cell_pos in grid.iter_circle(
            &(Circle {
                center: pos,
                radius: range,
            }),
        ) {
            let angle = (cell_pos - pos).angle() - angle;
            let dist = (cell_pos - pos).length();
            if dist > lidar.interpolate(angle) {
                continue;
            }

            // Skip cells which are out of bounds
            let Some(cell) = grid.get(cell_pos) else {
                continue;
            };

            // The relative position of the cell to the robot
            let vec = cell_pos - pos;

            // Weight is the difference between the cell and the robot heat
            let weight = cell - robot_heat;

            // Ignore cells that are colder than the robot as this ends up
            // repelling the robot leading to situations where the robot
            // gets stuck at the edge of the map
            if weight <= 0.0 {
                continue;
            }

            // We want the weight to be stronger the closer we are to the robot
            let nearness = 1.0 - vec.length() / range;
            let weight = weight * nearness;

            gradient += vec.normalized() * weight;

            cells.push((cell_pos, weight));
        }

        if !cells.is_empty() {
            gradient /= cells.len() as f32;
        }

        gradient *= GRADIENT_WEIGHT;
    }

    (gradient, cells)
}

fn search(
    robot: &mut RobotRef,
    time: Duration,
    update: fn(&mut SearchRobot, time: Duration),
    contributions: fn(&mut SearchRobot, &Vec2) -> Option<Vec2>,
) -> BehaviorOutput {
    let robot = cast_robot::<SearchRobot>(robot);

    // Debug visualization
    robot.visualize();

    update(robot, time);

    let forward_bias = Vec2::angled(robot.angle) * FORWARD_BIAS;
    robot.debug("", "Forward Bias", DebugType::Vector(forward_bias));

    let mut target = Vec2::ZERO;
    target += forward_bias;
    let target = contributions(robot, &target);

    robot.debug("", "Target", DebugType::Vector(target.unwrap_or_default()));

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
            |robot, target| match robot.robot_mode {
                RobotMode::Exploring => {
                    let mut target = *target;
                    target += robot.search_gradient();
                    target += robot.proximity_gradient();
                    // target += robot.lidar();
                    Some(target)
                }
                RobotMode::Pathing => {
                    // No += since we want full control
                    robot.path_planning()
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
            |robot, target| {
                let mut target = *target;
                target += robot.search_gradient();
                target += robot.proximity_gradient();
                Some(target)
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
            |robot, target| {
                let mut target = *target;
                target += robot.search_gradient();
                Some(target)
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
            |robot, target| {
                let mut target = *target;
                target += robot.search_gradient();
                target += robot.proximity_gradient();
                Some(target)
            },
        )
    }
}
