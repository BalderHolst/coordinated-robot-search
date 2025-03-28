//! This module contains the robot behaviors.

mod avoid_obstacles;
mod dumb;
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
}

impl RobotKind {
    /// Get the menu of behaviors for the robot kind.
    fn menu(&self) -> &[(&'static str, BehaviorFn)] {
        match self {
            RobotKind::Dumb => dumb::MENU,
            RobotKind::AvoidObstacles => avoid_obstacles::MENU,
            RobotKind::Search => search::MENU,
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
    robot_kind: RobotKind,
    behavior_name: &'static str,
    behavior_fn: BehaviorFn,
}

impl Behavior {
    /// Get the behavior function
    pub fn behavior_fn(&self) -> BehaviorFn {
        self.behavior_fn
    }

    pub fn create_fn(&self) -> fn() -> Box<dyn Robot> {
        match &self.robot_kind {
            RobotKind::Dumb => || Box::new(dumb::DumbRobot::default()),
            RobotKind::AvoidObstacles => {
                || Box::new(avoid_obstacles::AvoidObstaclesRobot::default())
            }
            RobotKind::Search => || Box::new(search::SearchRobot::default()),
        }
    }

    /// Create a new robot that corresponds to the behavior.
    /// This is the main way to create a `Box<dyn Robot>`.
    pub fn create_robot(&self) -> Box<dyn Robot> {
        (self.create_fn())()
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
            return Ok(Self {
                robot_kind: RobotKind::from_str(s, false)?,
                behavior_name,
                behavior_fn,
            });
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

        Ok(Self {
            robot_kind,
            behavior_name,
            behavior_fn,
        })
    }

    pub fn name(&self) -> String {
        let kind_name = self.robot_kind.to_possible_value().unwrap();
        format!("{}:{}", kind_name.get_name(), self.behavior_name)
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
        format!("{}:{}", self.robot_kind, self.behavior_name).serialize(serializer)
    }
}
