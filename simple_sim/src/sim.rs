use std::f32::consts::PI;

use eframe::{
    egui::{Color32, Pos2, Vec2},
    epaint::Hsva,
};
use robcore::{self, CamPoint};

use crate::grid::Grid;

const LIDAR_RAYS: usize = 40;
const LIDAR_RANGE: f32 = 5.0;

const CAMERA_RAYS: usize = 20;
const CAMERA_RANGE: f32 = 3.0;
pub const CAMERA_FOV: f32 = PI / 2.0;

const RAY_CAST_STEP: f32 = 0.5;
const CELLS_PR_METER: f32 = 50.0;

#[derive(Clone, Copy, Default, PartialEq)]
pub enum Cell {
    #[default]
    Empty,
    Wall,
    SearchItem,
    OutOfBounds,
}

impl Cell {
    pub fn is_empty(&self) -> bool {
        matches!(self, Self::Empty)
    }

    pub fn color(&self) -> Color32 {
        match self {
            Self::Empty => Color32::TRANSPARENT,
            Self::Wall => Hsva::new(0.6, 0.7, 0.5, 1.0).into(),
            Self::SearchItem => Hsva::new(0.22, 0.8, 0.8, 1.0).into(),
            Self::OutOfBounds => Color32::TRANSPARENT,
        }
    }
}

#[derive(Clone)]
pub struct World {
    grid: Grid<Cell>,
    width: f32,
    height: f32,
}

impl World {

    pub fn new(width: f32, height: f32) -> Self {
        let grid_width = (width * CELLS_PR_METER).ceil() as usize;
        let grid_height = (height * CELLS_PR_METER).ceil() as usize;

        Self {
            grid: Grid::new(grid_width, grid_height),
            width,
            height,
        }
    }

    pub fn size(&self) -> Vec2 {
        Vec2::new(self.width, self.height)
    }

    pub fn bounds(&self) -> (Pos2, Pos2) {
        let min = Pos2 {
            x: -self.width / 2.0,
            y: -self.height / 2.0,
        };
        let max = Pos2 {
            x: self.width / 2.0,
            y: self.height / 2.0,
        };
        (min, max)
    }

    fn world_to_grid(&self, pos: Pos2) -> Pos2 {
        ((pos + self.size() / 2.0) * CELLS_PR_METER).floor()
    }

    fn grid_to_world(&self, pos: Pos2) -> Pos2 {
        (pos + Vec2::splat(0.5)) / CELLS_PR_METER - self.size() / 2.0
    }

    pub fn get_cell(&self, pos: Pos2) -> Cell {
        if pos.x < self.width / -2.0
            || pos.x > self.width / 2.0
            || pos.y < self.height / -2.0
            || pos.y > self.height / 2.0
        {
            return Cell::OutOfBounds;
        }
        let pos = self.world_to_grid(pos);
        debug_assert!(pos.x >= 0.0, "x: {}", pos.x);
        debug_assert!(pos.y >= 0.0, "y: {}", pos.y);
        let x = pos.x as usize;
        let y = pos.y as usize;
        debug_assert!(
            x <= self.grid.width(),
            "x: {}, width: {}",
            x,
            self.grid.width()
        );
        debug_assert!(
            y <= self.grid.height(),
            "y: {}, height: {}",
            y,
            self.grid.height()
        );
        self.grid.get(x, y)
    }

    pub fn width(&self) -> f32 {
        self.width
    }

    pub fn height(&self) -> f32 {
        self.height
    }

    pub fn grid(&self) -> &Grid<Cell> {
        &self.grid
    }

    pub fn line(&mut self, start: Pos2, end: Pos2, width: f32, cell: Cell) {
        let width = width * CELLS_PR_METER;
        let start = self.world_to_grid(start);
        let end = self.world_to_grid(end);
        self.grid.line(start, end, width, cell);
    }

    pub fn circle(&mut self, center: Pos2, radius: f32, cell: Cell) {
        let radius = radius * CELLS_PR_METER;
        let center = self.world_to_grid(center);
        self.grid.circle(center, radius, cell);
    }
}

#[derive(Clone)]
pub struct Robot {
    pub pos: Pos2,
    pub vel: f32,
    pub angle: f32,
    pub avel: f32,
    pub lidar: robcore::LidarData,
    pub cam: robcore::CamData,
}

impl Robot {
    pub fn new_at(x: f32, y: f32, angle: f32) -> Self {
        Self {
            pos: Pos2 { x, y },
            angle,
            vel: 0.0,
            avel: 0.0,
            lidar: robcore::LidarData(vec![]),
            cam: robcore::CamData(vec![]),
        }
    }
}

impl robcore::Robot for Robot {
    fn get_pos(&self) -> robcore::Pos2 {
        robcore::Pos2 {
            x: self.pos.x,
            y: self.pos.y,
        }
    }

    fn get_cam_data(&self) -> &robcore::CamData {
        &self.cam
    }

    fn get_lidar_data(&self) -> &robcore::LidarData {
        &self.lidar
    }

    fn post(&self, msg: robcore::Message) {
        todo!()
    }

    fn recv(&mut self) -> Vec<robcore::Message> {
        todo!()
    }

    fn set_control(&mut self, control: robcore::Control) {
        self.vel = control.speed;
        self.avel = control.steer;
    }
}

#[derive(Clone)]
pub struct Simulator {
    pub robots: Vec<Robot>,
    pub robot_radius: f32,
    pub world: World,
    pub sps: f32,
    behavior: for<'a> fn(&'a mut (dyn robcore::Robot + 'a)),
}

impl Simulator {
    pub fn new(
        world: World,
        sps: f32,
        behavior: for<'a> fn(&'a mut (dyn robcore::Robot + 'a)),
    ) -> Self {
        Self {
            robots: vec![],
            robot_radius: 0.3,
            behavior,
            world,
            sps,
        }
    }

    fn dt(&self) -> f32 {
        1.0 / self.sps
    }

    pub fn add_robot(&mut self, robot: Robot) {
        self.robots.push(robot);
    }

    fn cast_ray(&self, pos: Pos2, angle: f32, max_range: f32) -> (f32, Option<Cell>) {
        let mut distance = 0.0;
        let mut pos = pos;
        let step_size = RAY_CAST_STEP / CELLS_PR_METER;
        let step = step_size * Vec2::angled(angle);

        while distance < max_range {
            let cell = self.world.get_cell(pos);
            if !cell.is_empty() {
                return (distance, Some(cell));
            }

            // Check for collisions with other robots
            if distance > self.robot_radius {
                for robot in &self.robots {
                    if (robot.pos - pos).length() < self.robot_radius {
                        return (distance, None);
                    }
                }
            }

            pos += step;
            distance += step_size;
        }

        (distance, None)
    }

    fn resolve_robot_collisions(&mut self) {
        for i in 0..self.robots.len() {
            for j in i + 1..self.robots.len() {
                let robot1 = &self.robots[i];
                let robot2 = &self.robots[j];
                if (robot1.pos - robot2.pos).length() < self.robot_radius * 2.0 {
                    let diff = robot1.pos - robot2.pos;
                    let overlap = self.robot_radius * 2.0 - diff.length();
                    let dir = diff.normalized() * overlap / 2.0;
                    self.robots[i].pos += dir;
                    self.robots[j].pos -= dir;
                }
            }
        }
    }

    fn resolve_world_collisions(&mut self) {
        for robot in &mut self.robots {
            // Look in a circle around the robot
            let radius = self.robot_radius * CELLS_PR_METER * 1.4;

            let center = self.world.world_to_grid(robot.pos);
            let mut nudge = Vec2::ZERO;
            let mut nudgers = 0;
            for (x, y) in self.world.grid.circle_iter(center, radius) {
                let cell = self.world.grid.get(x, y);
                if !cell.is_empty() {
                    let cell_center = self.world.grid_to_world(Pos2 {
                        x: x as f32,
                        y: y as f32,
                    });
                    let diff = robot.pos - cell_center;

                    let overlap = self.robot_radius - diff.length() + 0.5 / CELLS_PR_METER;
                    if overlap < 0.0 {
                        continue;
                    }

                    let diff = match diff.x.abs() > diff.y.abs() {
                        true => Vec2::new(diff.x, 0.0),
                        false => Vec2::new(0.0, diff.y),
                    };
                    let dir = diff.normalized();

                    nudge += dir * overlap;
                    nudgers += 1;
                }
            }

            if nudgers > 0 {
                // Only nudge in the direction with most difference
                robot.pos += nudge / nudgers as f32;
            }
        }
    }

    fn resolve_border_collisions(&mut self) {
        for robot in &mut self.robots {
            if robot.pos.x - self.robot_radius < -self.world.width() / 2.0 {
                robot.pos.x = -self.world.width() / 2.0 + self.robot_radius;
            }
            if robot.pos.x + self.robot_radius > self.world.width() / 2.0 {
                robot.pos.x = self.world.width() / 2.0 - self.robot_radius;
            }
            if robot.pos.y - self.robot_radius < -self.world.height() / 2.0 {
                robot.pos.y = -self.world.height() / 2.0 + self.robot_radius;
            }
            if robot.pos.y + self.robot_radius > self.world.height() / 2.0 {
                robot.pos.y = self.world.height() / 2.0 - self.robot_radius;
            }
        }
    }

    pub fn step(&mut self) {
        let dt = self.dt();

        // Call the behavior function for each robot
        for robot in &mut self.robots {
            (self.behavior)(robot);
        }

        // Update the position of each robot
        for robot in self.robots.iter_mut() {
            let vel = Vec2::angled(robot.angle) * robot.vel;

            robot.pos += vel * dt;
            robot.angle += robot.avel * dt;
        }

        self.resolve_robot_collisions();
        self.resolve_border_collisions();
        self.resolve_world_collisions();

        // Update robot lidar data
        let mut lidar_data = Vec::with_capacity(self.robots.len());
        for robot in &self.robots {
            let points = (0..LIDAR_RAYS)
                .map(|n| {
                    let angle = n as f32 / LIDAR_RAYS as f32 * 2.0 * PI;
                    let (distance, _) = self.cast_ray(robot.pos, robot.angle + angle, LIDAR_RANGE);
                    robcore::LidarPoint { angle, distance }
                })
                .collect();
            lidar_data.push(robcore::LidarData(points));
        }

        // Update robot camera
        let mut cam_data = Vec::with_capacity(self.robots.len());

        const ANGLE_STEP: f32 = CAMERA_FOV / (CAMERA_RAYS - 1) as f32;

        for robot in &self.robots {
            let points = (0..CAMERA_RAYS)
                .filter_map(|n| {
                    let angle = n as f32 * ANGLE_STEP - CAMERA_FOV / 2.0;
                    let (distance, cell) = self.cast_ray(robot.pos, robot.angle + angle, CAMERA_RANGE);
                    match cell {
                        Some(Cell::SearchItem) => {
                            let propability = (CAMERA_RANGE - distance) / CAMERA_RANGE;
                            Some((n, robcore::CamPoint { angle, propability }))
                        }
                        _ => None,
                    }
                })
                .collect::<Vec<_>>();

            // Consolidate points next to each other
            {
                let mut sparse_points = vec![];
                let mut adjacant = vec![];

                let consolidate_points = |adjacant: &mut Vec<_>, sparse_points: &mut Vec<_>| {
                    if adjacant.is_empty() {
                        return;
                    }

                    let mut avg_point = adjacant.iter().fold(
                        CamPoint {
                            angle: 0.0,
                            propability: 0.0,
                        },
                        |a: CamPoint, (_, b): &(usize, CamPoint)| CamPoint {
                            angle: a.angle + b.angle,
                            propability: a.propability + b.propability,
                        },
                    );
                    avg_point.propability /= adjacant.len() as f32;
                    avg_point.angle /= adjacant.len() as f32;
                    sparse_points.push(avg_point);
                    adjacant.clear();
                };

                for (n, point) in points {
                    match adjacant.last() {
                        None => adjacant.push((n, point)),
                        Some((last_n, _)) if *last_n == n - 1 => adjacant.push((n, point)),
                        Some(_) => consolidate_points(&mut adjacant, &mut sparse_points),
                    }
                }
                consolidate_points(&mut adjacant, &mut sparse_points);
                cam_data.push(robcore::CamData(sparse_points));
            }
        }

        // Actually update the robots
        let data = lidar_data.into_iter().zip(cam_data);
        for (robot, (lidar, cam)) in self.robots.iter_mut().zip(data) {
            robot.lidar = lidar;
            robot.cam = cam;
        }
    }
}
