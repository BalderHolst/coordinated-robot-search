pub mod model;
pub mod state;

use std::{collections::HashMap, time::Duration};

use emath::Pos2;
use state::RlState;

use crate::{
    debug::DebugSoup, scaled_grid::ScaledGrid, CamData, LidarData, Postbox, Robot, RobotId,
    RobotPose, Vec2,
};

use super::{cast_robot, BehaviorFn, BehaviorOutput, RobotRef};

pub const MENU: &[(&str, BehaviorFn)] = &[("nn", run_nn)];

const SEARCH_GRID_SCALE: f32 = 0.20;

type MyBackend = burn::backend::Wgpu;

#[derive(Clone)]
pub struct RlRobot {
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

    /// The time of the last search grid update
    pub last_search_grid_update: Duration,

    /// Grid containing position preferences for the robot based on
    /// the positions of other robots
    pub proximity_grid: ScaledGrid<f32>,

    /// Other robots and their positions
    pub others: HashMap<RobotId, (Pos2, f32)>,

    /// Debug object and their names. Used for visualization.
    /// Set to `None` to disable debug visualization.
    pub debug_soup: DebugSoup,

    /// The neural network used to control the robot. It is protexted
    /// by a `RwLock` to allow multiple threads to read the model and
    /// for the model to be dynamically updated when training.
    pub model: model::BotModel<MyBackend>,
}

impl Default for RlRobot {
    fn default() -> Self {
        Self {
            id: Default::default(),
            pos: Default::default(),
            angle: Default::default(),
            cam: Default::default(),
            lidar: Default::default(),
            postbox: Default::default(),
            search_grid: Default::default(),
            last_search_grid_update: Default::default(),
            proximity_grid: Default::default(),
            others: Default::default(),
            debug_soup: Default::default(),
            model: model::BotModel::new_model(),
        }
    }
}

impl Robot for RlRobot {
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

    fn any(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

impl RlRobot {
    pub fn state(&self) -> RlState<MyBackend> {
        RlState::new(self.pos, self.angle, self.lidar.clone())
    }

    pub fn from_model(model: model::ModelRef<MyBackend>) -> Self {
        Self {
            model: model::BotModel::new_ref(model),
            ..Default::default()
        }
    }
}

fn run_nn(robot: &mut RobotRef, _time: Duration) -> BehaviorOutput {
    let _robot = cast_robot::<RlRobot>(robot);
    todo!();
}
