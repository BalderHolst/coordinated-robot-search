mod action;
pub mod model;
pub mod state;

use std::{collections::HashMap, marker::PhantomData, time::Duration};

use action::RlAction;
use emath::Pos2;
use model::small::SmallNetwork;
use state::State;

use crate::{
    debug::{DebugSoup, DebugType},
    scaled_grid::ScaledGrid,
    CamData, Control, LidarData, Postbox, Robot, RobotId, RobotPose, Vec2,
};

use super::{cast_robot, common, params, BehaviorFn, BehaviorOutput, RobotRef};

pub const MENU: &[(&str, BehaviorFn)] = &[("nn", small)];

const SEARCH_GRADIENT_RANGE: f32 = 5.0;
const SEARCH_GRID_SCALE: f32 = 0.20;
const SEARCH_GRID_UPDATE_INTERVAL: f32 = 0.1;

/// The frequency at which the robot reacts to the environment
pub const REACT_HZ: f32 = 2.0;

type MyBackend = burn::backend::Wgpu;

#[derive(Clone)]
pub struct RlRobot<S: State> {
    _state: PhantomData<S>,

    /// The id of the robot
    pub id: RobotId,

    /// The position of the robot
    pub pos: Pos2,

    /// The angle of the robot
    pub angle: f32,

    /// The data from the camera. Angles and probability of objects.
    pub cam: CamData,

    /// The data from the lidar. Distance to objects.
    pub lidar: LidarData,

    pub postbox: Postbox,

    /// Grid containing probabilities of objects in the environment.
    pub search_grid: ScaledGrid<f32>,

    pub search_gradient: Vec2,

    /// The time of the last search grid update
    pub last_search_grid_update: Duration,

    /// Other robots and their positions
    pub others: HashMap<RobotId, (Pos2, f32)>,

    /// Debug object and their names. Used for visualization.
    /// Set to `None` to disable debug visualization.
    pub debug_soup: DebugSoup,

    /// The neural network used to control the robot. It is protexted by a `RwLock` to allow multiple threads to read the model and
    /// for the model to be dynamically updated when training.
    pub model: model::BotModel<MyBackend, S, SmallNetwork<MyBackend>>,

    /// Last time the robot reacted to the environment
    pub last_control_update: Duration,

    /// The current control signal
    pub control: Control,
}

impl<S: State + 'static> Robot for RlRobot<S> {
    fn get_id(&self) -> &RobotId {
        &self.id
    }

    fn set_id(&mut self, id: RobotId) {
        self.id = id;
    }

    fn set_world_size(&mut self, size: Vec2) {
        self.search_grid = ScaledGrid::new(size.x, size.y, SEARCH_GRID_SCALE);
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

impl<S: State + 'static> RlRobot<S> {
    pub fn new() -> Self {
        Self {
            id: RobotId(0),
            pos: Pos2::ZERO,
            angle: 0.0,
            cam: CamData::default(),
            lidar: LidarData::default(),
            postbox: Default::default(),
            search_grid: Default::default(),
            last_search_grid_update: Default::default(),
            search_gradient: Default::default(),
            others: Default::default(),
            debug_soup: DebugSoup::new_active(),
            model: model::BotModel::new_model(&Default::default()),
            last_control_update: Duration::ZERO,
            control: Default::default(),
            _state: PhantomData,
        }
    }

    /// Get the state used as input to the neural network
    pub fn state(&self) -> S {
        S::from(self.clone())
    }

    /// React to the environment and return a control signal
    pub fn react(&mut self) {
        let input = self.state().to_tensor();
        let action = self.model.action(input);
        self.control = action.control();
    }

    pub fn new_controlled() -> Self {
        Self {
            model: model::BotModel::new_controlled(RlAction::default()),
            ..Self::new()
        }
    }

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
}

impl RlRobot<state::SmallState> {
    fn visualize(&mut self) {
        if !self.debug_enabled() {
            return;
        }
        self.debug("", "Search Grid", DebugType::Grid(self.search_grid.clone()));

        let raw_lidar = self.lidar.points().map(|p| (p.angle, p.distance)).collect();
        self.debug("", "Actual Lidar", DebugType::RobotRays(raw_lidar));

        let interpolated_lidar = self.state().lidar_rays().to_vec();

        if let Some(shortest_ray) = interpolated_lidar
            .iter()
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .cloned()
        {
            self.debug(
                "",
                "Shortest Lidar Angle",
                DebugType::Number(shortest_ray.0),
            );
            self.debug(
                "",
                "Shortest Lidar Distance",
                DebugType::Number(shortest_ray.1),
            );
            self.debug(
                "",
                "Shortest Ray",
                DebugType::RobotRays(vec![(shortest_ray.0, shortest_ray.1)]),
            );
        }

        self.debug(
            "",
            "Interpolated Lidar",
            DebugType::RobotRays(interpolated_lidar),
        );
    }

    fn update_search_gradient(&mut self) {
        let (g, _) = common::gradient(
            self.pos,
            self.angle,
            SEARCH_GRADIENT_RANGE,
            params::DIAMETER * 2.0,
            self.lidar.clone(),
            &self.search_grid,
        );
        self.debug("Search Gradient", "Vector", DebugType::Vector(g));
        self.debug("Search Gradient", "x", DebugType::Number(g.x));
        self.debug("Search Gradient", "y", DebugType::Number(g.y));
        self.debug("Search Gradient", "length", DebugType::Number(g.length()));
        self.debug("Search Gradient", "angle", DebugType::Number(g.angle()));
        self.search_gradient = g;
    }
}

fn small(robot: &mut RobotRef, time: Duration) -> BehaviorOutput {
    let robot = cast_robot::<RlRobot<state::SmallState>>(robot);

    robot.visualize();

    robot.update_search_grid(time);
    robot.update_search_gradient();

    // Only update the control signal at a fixed rate
    if (time - robot.last_control_update).as_secs_f32() >= 1.0 / REACT_HZ
        || robot.model.is_controlled()
    {
        robot.last_control_update = time;
        robot.react();
    }

    robot.postbox.clean();

    let msgs = robot.postbox.empty();
    (robot.control.clone(), msgs)
}
