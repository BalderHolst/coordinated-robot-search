//! This module contains the robot behaviors.

mod avoid_obstacles;
mod dumb;
mod search;

use std::time::Instant;

#[cfg(feature = "cli")]
use clap::ValueEnum;

use super::*;

pub type BehaviorFn = fn(&mut Box<dyn Robot>, Instant) -> Control;

#[cfg_attr(feature = "cli", derive(clap::ValueEnum))]
#[derive(Clone, Debug)]
pub enum RobotKind {
    Dumb,
    AvoidObstacles,
    Search,
}

impl RobotKind {
    fn menu(&self) -> &[(&'static str, BehaviorFn)] {
        match self {
            RobotKind::Dumb => dumb::MENU,
            RobotKind::AvoidObstacles => avoid_obstacles::MENU,
            RobotKind::Search => search::MENU,
        }
    }
}
#[derive(Clone, Debug)]
pub struct Behavior {
    robot_kind: RobotKind,
    behavior_name: &'static str,
    behavior_fn: BehaviorFn,
}

impl Behavior {
    pub fn behavior_fn(&self) -> BehaviorFn {
        self.behavior_fn
    }

    pub fn create_robot(&self) -> Box<dyn Robot> {
        match &self.robot_kind {
            RobotKind::Dumb => Box::new(dumb::DumbRobot::default()),
            RobotKind::AvoidObstacles => Box::new(avoid_obstacles::AvoidObstaclesRobot::default()),
            RobotKind::Search => Box::new(search::SearchRobot::default()),
        }
    }
}

#[cfg(feature = "cli")]
impl Behavior {
    pub fn behavior_names() -> Vec<String> {
        RobotKind::value_variants()
            .into_iter()
            .map(|b| {
                let robot_kind_name = b.to_possible_value().unwrap().get_name().to_string();
                b.menu()
                    .iter()
                    .map(move |(behavior_name, _)| format!("{robot_kind_name}:{behavior_name}"))
            })
            .flatten()
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
            .into_iter()
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
}

#[cfg(feature = "cli")]
impl clap::builder::ValueParserFactory for Behavior {
    type Parser = clap::builder::ValueParser;
    fn value_parser() -> Self::Parser {
        clap::builder::ValueParser::new(Behavior::parse)
    }
}
