//! This module contains the robot behaviors.

mod avoid_obstacles;
mod common;
mod dumb;
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

pub type Time = Duration;
pub type BehaviorOutput = (Control, Vec<Message>);
pub type BehaviorFn = fn(&mut Box<dyn Robot>, Time) -> BehaviorOutput;
pub type CreateFn = fn() -> Box<dyn Robot>;

/// The kind of robot. Behaviors can only be run the robot kind they were designed for.
#[cfg_attr(feature = "cli", derive(clap::ValueEnum))]
#[derive(Clone, Debug)]
pub enum RobotKind {
    Dumb,
    AvoidObstacles,
    Search,
    Rl,
}

impl RobotKind {
    /// Get the menu of behaviors for the robot kind.
    fn menu(&self) -> &[(&'static str, BehaviorFn)] {
        match self {
            RobotKind::Dumb => dumb::MENU,
            RobotKind::AvoidObstacles => avoid_obstacles::MENU,
            RobotKind::Search => search::MENU,
            RobotKind::Rl => rl::MENU,
        }
    }

    pub fn create_fn(&self) -> CreateFn {
        match &self {
            RobotKind::Dumb => || Box::new(dumb::DumbRobot::default()),
            RobotKind::AvoidObstacles => {
                || Box::new(avoid_obstacles::AvoidObstaclesRobot::default())
            }
            RobotKind::Search => || Box::new(search::SearchRobot::default()),
            RobotKind::Rl => || Box::new(rl::RlRobot::<rl::state::SmallState>::new()),
        }
    }

    fn get_name(&self) -> &'static str {
        match self {
            RobotKind::Dumb => "dumb",
            RobotKind::AvoidObstacles => "avoid-obstacles",
            RobotKind::Search => "search",
            RobotKind::Rl => "rl",
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

    /// Get the name of the behavior and robot. Format: "<robot>:<behavior>".
    pub fn name(&self) -> String {
        format!("{}:{}", self.robot_name, self.behavior_name)
    }
}

#[cfg(feature = "cli")]
impl Behavior {
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
