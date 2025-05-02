//! Implementation of Reinforcement Learning (RL) robots
//!
//! The [`RlRobot`] struct is highly generic. Which makes it
//! possible to compose and reuse the state, actions and network types.
//!
//! States, actions and networks are defined in their own submodules.
//!
//! Robots are defined in the [`robots`] module, and are a specialization
//! of the [`RlRobot`] struct.

/// Implementation of a robot using reinforcement learning robots.
pub mod action;
pub mod network;
pub mod robots;
pub mod state;

use std::{collections::HashMap, time::Duration};

use action::Action;
use burn::prelude::Backend;
use emath::Pos2;
use network::{Model, Network, TrainedNetwork};
use state::State;

use crate::{
    debug_soup::DebugSoup, scaled_grid::ScaledGrid, CamData, Control, LidarData, Map, Postbox,
    Robot, RobotId, RobotPose, Vec2,
};

use super::{common, params, DebugItem};

const SEARCH_GRADIENT_RANGE: f32 = 5.0;
const SEARCH_GRID_SCALE: f32 = 0.20;
const SEARCH_GRID_UPDATE_INTERVAL: f32 = 0.1;

/// The frequency at which the robot reacts to the environment
pub const REACT_HZ: f32 = 2.0;

/// The generic RL robot struct
#[derive(Clone)]
pub struct RlRobot<B: Backend, S: State, A: Action, N: Network<B, S, A>> {
    /// The id of the robot
    id: RobotId,

    /// The position of the robot
    pos: Pos2,

    /// The angle of the robot
    angle: f32,

    /// The data from the camera. Angles and probability of objects.
    cam: CamData,

    /// The data from the lidar. Distance to objects.
    pub lidar: LidarData,

    postbox: Postbox,

    map: Map,

    /// Grid containing probabilities of objects in the environment.
    search_grid: ScaledGrid<f32>,

    search_gradient: Vec2,

    /// The time of the last search grid update
    last_search_grid_update: Duration,

    /// Other robots and their positions
    others: HashMap<RobotId, (Pos2, f32)>,

    /// Debug object and their names. Used for visualization.
    debug_soup: DebugSoup,

    /// The neural network used to control the robot. It is protexted by a `RwLock` to allow multiple threads to read the model and
    /// for the model to be dynamically updated when training.
    pub model: network::BotModel<B, S, A, N>,

    /// Last time the robot reacted to the environment
    last_control_update: Duration,

    /// The current control signal
    control: Control,
}

impl<B: Backend, S: State, A: Action, N: Network<B, S, A>> Robot for RlRobot<B, S, A, N> {
    fn get_id(&self) -> &RobotId {
        &self.id
    }

    fn set_id(&mut self, id: RobotId) {
        self.id = id;
    }

    fn set_map(&mut self, map: super::Map) {
        let size = map.size();
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

impl<B: Backend, S: State, A: Action, N: Network<B, S, A>> Default for RlRobot<B, S, A, N> {
    fn default() -> Self {
        Self {
            id: RobotId(0),
            pos: Pos2::ZERO,
            angle: 0.0,
            cam: CamData::default(),
            lidar: LidarData::default(),
            postbox: Default::default(),
            map: Default::default(),
            search_grid: Default::default(),
            last_search_grid_update: Default::default(),
            search_gradient: Default::default(),
            others: Default::default(),
            debug_soup: DebugSoup::new_active(),
            model: network::BotModel::new_model(Model::<B, S, A, N>::new(N::init(
                &Default::default(),
            ))),
            last_control_update: Duration::ZERO,
            control: Default::default(),
        }
    }
}

impl<B: Backend, S: State, A: Action, N: TrainedNetwork<B, S, A>> RlRobot<B, S, A, N> {
    /// Creates a new RL robot with a trained model.
    pub fn new_trained() -> Self {
        Self {
            model: network::BotModel::new_trained_model(&Default::default()),
            ..Default::default()
        }
    }
}

impl<B: Backend, S: State, A: Action, N: Network<B, S, A>> RlRobot<B, S, A, N> {
    /// Creates a new RL robot with a new random model.
    pub fn new_network(device: &B::Device) -> N {
        N::init(device)
    }

    /// Creates a new controlled RL robot
    pub fn new_controlled() -> Self {
        Self {
            model: network::BotModel::new_controlled(A::default()),
            ..Default::default()
        }
    }

    /// Get the state used as input to the neural network
    pub fn state(&self) -> S {
        S::from_robot(self)
    }

    /// React to the environment and return a control signal
    pub fn react(&mut self) {
        let input = self.state().to_tensor(&Default::default());
        let action = self.model.action(input);
        self.control = action.control();
    }

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

    fn update_search_gradient(&mut self) {
        let (g, _) = common::gradient(
            self.pos,
            self.angle,
            SEARCH_GRADIENT_RANGE,
            params::DIAMETER * 2.0,
            self.lidar.clone(),
            &self.search_grid,
        );
        self.debug("Search Gradient", "Vector", DebugItem::Vector(g));
        self.debug("Search Gradient", "x", DebugItem::Number(g.x));
        self.debug("Search Gradient", "y", DebugItem::Number(g.y));
        self.debug("Search Gradient", "length", DebugItem::Number(g.length()));
        self.debug("Search Gradient", "angle", DebugItem::Number(g.angle()));
        self.search_gradient = g;
    }

    fn visualize(&mut self) {
        if !self.debug_enabled() {
            return;
        }
        self.debug("", "Search Grid", DebugItem::Grid(self.search_grid.clone()));

        let raw_lidar = self.lidar.points().map(|p| (p.angle, p.distance)).collect();
        self.debug("", "Lidar", DebugItem::RobotRays(raw_lidar));
    }
}
