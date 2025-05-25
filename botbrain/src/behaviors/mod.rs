//! This module contains the robot behaviors.

mod avoid_obstacles;
mod common;
mod dumb;
#[cfg(feature = "rl")]
pub mod rl;
mod search;

use std::time::Duration;

#[cfg(feature = "cli")]
use {
    clap::ValueEnum,
    serde::Deserializer,
    std::fmt::{self, Display},
};

use super::*;

/// A function that runs a behavior on a robot.
/// Takes a robot and a time and returns a control signal and a list of messages.
pub type BehaviorFn = fn(&mut RobotRef, Time) -> BehaviorOutput;

/// A function that creates a new robot.
pub type CreateFn = fn() -> RobotRef;

/// The current time since the creation of the robot
pub type Time = Duration;

/// The output of a [BehaviorFn].
pub type BehaviorOutput = (Control, Vec<Message>);

#[cfg(feature = "rl")]
type MyBackend = burn::backend::Wgpu;

/// The kind of robot. Behaviors can only be run the robot kind they were designed for.
#[cfg_attr(feature = "cli", derive(clap::ValueEnum))]
#[derive(Clone, Debug)]
pub enum RobotKind {
    /// Very dumb robot that uses no state
    Dumb,

    /// Robot that only uses its lidar
    AvoidObstacles,

    /// Robot capable of searching an environment for an object
    Search,

    /// Reinforcement learning using only the minimal state, action and network to avoid obstacles
    #[cfg(feature = "rl")]
    MinimalRl,

    /// Reinforcement learning using polar coordinates in its state. Uses a small sized network.
    #[cfg(feature = "rl")]
    SmallPolarRl,

    /// Reinforcement learning using a small state with polar coordinates. Uses a single layer network.
    #[cfg(feature = "rl")]
    TinyPolarRl,

    /// Reinforcement learning using polar coordinates in its state. Uses a medium sized network.
    #[cfg(feature = "rl")]
    MediumPolarRl,

    /// A reinforcement learning robot using a small network
    #[cfg(feature = "rl")]
    SmallRl,
    // /// Reinforcement learning using big network.
    // #[cfg(feature = "rl")]
    // BigBoyRl,
}

impl RobotKind {
    /// Get the menu of behaviors for the robot kind.
    pub fn menu(&self) -> &[(&'static str, BehaviorFn)] {
        match self {
            RobotKind::Dumb => dumb::MENU,
            RobotKind::AvoidObstacles => avoid_obstacles::MENU,
            RobotKind::Search => search::MENU,
            #[cfg(feature = "rl")]
            RobotKind::SmallRl => &[("run", rl::robots::small::run::<MyBackend>)],
            #[cfg(feature = "rl")]
            RobotKind::TinyPolarRl => &[("run", rl::robots::tiny_polar::run::<MyBackend>)],
            #[cfg(feature = "rl")]
            RobotKind::SmallPolarRl => &[("run", rl::robots::small_polar::run::<MyBackend>)],
            #[cfg(feature = "rl")]
            RobotKind::MinimalRl => &[("run", rl::robots::minimal::run::<MyBackend>)],
            #[cfg(feature = "rl")]
            RobotKind::MediumPolarRl => &[("run", rl::robots::medium_polar::run::<MyBackend>)],
            // #[cfg(feature = "rl")]
            // RobotKind::BigBoyRl => &[("run", rl::robots::bigboy::run::<MyBackend>)],
        }
    }

    /// Get the behavior function for the robot kind. Panics if the behavior is not found.
    pub fn get_behavior_fn(&self, behavior_name: &str) -> BehaviorFn {
        self.menu()
            .iter()
            .find(|(name, _)| *name == behavior_name)
            .map(|(_, f)| *f)
            .unwrap_or_else(|| {
                panic!(
                    "Behavior {} not found for robot kind {}. Valid behaviors: [{}]",
                    behavior_name,
                    self.get_name(),
                    self.menu()
                        .iter()
                        .map(|(name, _)| *name)
                        .collect::<Vec<_>>()
                        .join(", ")
                )
            })
    }

    /// Get the function used to create a new robot corresponding to the robot kind
    pub fn create_fn(&self) -> CreateFn {
        match &self {
            RobotKind::Dumb => || Box::new(dumb::DumbRobot::default()),
            RobotKind::AvoidObstacles => {
                || Box::new(avoid_obstacles::AvoidObstaclesRobot::default())
            }
            RobotKind::Search => || Box::new(search::SearchRobot::default()),
            #[cfg(feature = "rl")]
            RobotKind::SmallRl => {
                || Box::new(rl::robots::small::SmallRlRobot::<MyBackend>::new_trained())
            }
            #[cfg(feature = "rl")]
            RobotKind::SmallPolarRl => {
                || Box::new(rl::robots::small_polar::SmallPolarRlRobot::<MyBackend>::new_trained())
            }
            #[cfg(feature = "rl")]
            RobotKind::MediumPolarRl => || {
                Box::new(rl::robots::medium_polar::MediumPolarRlRobot::<MyBackend>::new_trained())
            },
            #[cfg(feature = "rl")]
            RobotKind::MinimalRl => {
                || Box::new(rl::robots::minimal::MinimalRlRobot::<MyBackend>::new_trained())
            }
            // #[cfg(feature = "rl")]
            // RobotKind::BigBoyRl => {
            //     || Box::new(rl::robots::bigboy::BigBoyRlRobot::<MyBackend>::default())
            // }
            #[cfg(feature = "rl")]
            RobotKind::TinyPolarRl => {
                || Box::new(rl::robots::tiny_polar::TinyPolarRlRobot::<MyBackend>::default())
            }
        }
    }

    /// Get the name of the robot kind
    pub fn get_name(&self) -> &'static str {
        match self {
            RobotKind::Dumb => "dumb",
            RobotKind::AvoidObstacles => "avoid-obstacles",
            RobotKind::Search => "search",
            #[cfg(feature = "rl")]
            RobotKind::SmallRl => "small-rl",
            #[cfg(feature = "rl")]
            RobotKind::TinyPolarRl => "tiny-polar-rl",
            #[cfg(feature = "rl")]
            RobotKind::SmallPolarRl => "small-polar-rl",
            #[cfg(feature = "rl")]
            RobotKind::MediumPolarRl => "medium-polar-rl",
            #[cfg(feature = "rl")]
            RobotKind::MinimalRl => "minimal-rl",
            // #[cfg(feature = "rl")]
            // RobotKind::BigBoyRl => "big-boy-rl",
        }
    }
}

#[cfg(feature = "cli")]
impl Display for RobotKind {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.to_possible_value().unwrap().get_name())
    }
}

/// A behavior defines the kind of robot and the behavior function to run on the robot.
#[derive(Clone, Debug)]
pub struct Behavior {
    robot_name: &'static str,
    behavior_name: &'static str,
    behavior_fn: BehaviorFn,
    create_fn: CreateFn,
}

impl Behavior {
    /// Create a new behavior from a robot kind and a behavior function.
    pub fn new(
        robot_kind: RobotKind,
        behavior_name: &'static str,
        behavior_fn: BehaviorFn,
    ) -> Self {
        Self {
            robot_name: robot_kind.get_name(),
            behavior_name,
            behavior_fn,
            create_fn: robot_kind.create_fn(),
        }
    }

    #[allow(missing_docs)]
    pub fn from_raw(
        robot_name: &'static str,
        behavior_name: &'static str,
        behavior_fn: BehaviorFn,
        create_fn: CreateFn,
    ) -> Self {
        Self {
            robot_name,
            behavior_name,
            behavior_fn,
            create_fn,
        }
    }

    /// Rename a behavior into a new one.
    pub fn with_name(self, name: &'static str) -> Self {
        Self {
            behavior_name: name,
            ..self
        }
    }

    /// Get the behavior function
    pub fn behavior_fn(&self) -> BehaviorFn {
        self.behavior_fn
    }

    /// Get the function used to create a new robot
    pub fn create_fn(&self) -> CreateFn {
        self.create_fn
    }

    /// Create a new robot that corresponds to the behavior.
    /// This is the main way to create a `Box<dyn Robot>`.
    pub fn create_robot(&self) -> Box<dyn Robot> {
        (self.create_fn)()
    }

    /// Get the name of the behavior and robot. Format: "{robot}:{behavior}".
    pub fn name(&self) -> String {
        format!("{}:{}", self.robot_name, self.behavior_name)
    }
}

#[cfg(feature = "cli")]
impl Behavior {
    /// Get the list of all possible behavior names.
    pub fn behavior_names() -> Vec<String> {
        RobotKind::value_variants()
            .iter()
            .flat_map(|b| {
                let robot_kind_name = b.to_possible_value().unwrap().get_name().to_string();
                b.menu()
                    .iter()
                    .map(move |(behavior_name, _)| format!("{robot_kind_name}:{behavior_name}"))
            })
            .collect::<Vec<_>>()
    }

    /// Parse a string into a behavior on the format "{robot}:{behavior}".
    pub fn parse(s: &str) -> Result<Self, String> {
        let Some((robot_kind_name, behavior_name)) = s.split_once(':') else {
            let robot_kind = RobotKind::from_str(s, true).map_err(|_| {
                format!(
                    "Valid behaviors: [{}]",
                    Behavior::behavior_names().join(", ")
                )
            })?;
            let (behavior_name, behavior_fn) = *robot_kind
                .menu()
                .first()
                .expect("Robots should have at least one behavior");

            return Ok(Self::new(robot_kind, behavior_name, behavior_fn));
        };

        let robot_kind = RobotKind::from_str(robot_kind_name, true)?;

        let (behavior_name, behavior_fn) = robot_kind
            .menu()
            .iter()
            .find(|(name, _)| *name == behavior_name)
            .map(|(n, f)| (*n, *f))
            .ok_or(format!(
                "Valid behaviors: {}",
                Self::behavior_names().join(", ")
            ))?;

        Ok(Self::new(robot_kind, behavior_name, behavior_fn))
    }
}

#[cfg(feature = "cli")]
impl clap::builder::ValueParserFactory for Behavior {
    type Parser = clap::builder::ValueParser;
    fn value_parser() -> Self::Parser {
        clap::builder::ValueParser::new(Behavior::parse)
    }
}

#[cfg(feature = "cli")]
impl<'de> Deserialize<'de> for Behavior {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;
        Behavior::parse(&s).map_err(serde::de::Error::custom)
    }
}

#[cfg(feature = "cli")]
impl Serialize for Behavior {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        self.name().serialize(serializer)
    }
}
