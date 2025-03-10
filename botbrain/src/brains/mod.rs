//! This module contains the robot behaviors.

mod avoid_obstacles;
mod dumb;
mod search;

use std::time::Instant;

#[cfg(feature = "cli")]
use clap::ValueEnum;

use super::*;

#[derive(Clone, Debug)]
pub struct Behavior {
    brain: Brain,
    behavior_name: &'static str,
    behavior_index: usize,
}

#[cfg_attr(feature = "cli", derive(clap::ValueEnum))]
#[derive(Clone, Debug)]
pub enum Brain {
    Dumb,
    AvoidObstacles,
    Search,
}

impl Behavior {
    pub fn create_robot(&self) -> Box<dyn Robot> {
        match &self.brain {
            Brain::Dumb => Box::new(dumb::DumbRobot::default()),
            Brain::AvoidObstacles => Box::new(avoid_obstacles::AvoidObstaclesRobot::default()),
            Brain::Search => Box::new(search::SearchRobot::default()),
        }
    }

    pub fn run(&self, robot: &mut Box<dyn Robot>, time: Instant) -> Control {
        robot.behavior(self.behavior_index, time)
    }
}

impl Brain {
    fn menu(&self) -> &[&'static str] {
        match self {
            Brain::Dumb => dumb::behaviors::MENU,
            Brain::AvoidObstacles => avoid_obstacles::MENU,
            Brain::Search => search::MENU,
        }
    }
}

#[cfg(feature = "cli")]
impl Behavior {
    pub fn behavior_names() -> Vec<String> {
        Brain::value_variants()
            .into_iter()
            .map(|b| {
                let brain_name = b.to_possible_value().unwrap().get_name().to_string();
                b.menu()
                    .iter()
                    .map(move |name| format!("{brain_name}:{name}"))
            })
            .flatten()
            .collect::<Vec<_>>()
    }

    pub fn parse(s: &str) -> Result<Self, String> {
        let Some((brain_name, behavior_name)) = s.split_once(':') else {
            let brain = Brain::from_str(s, true).map_err(|_| {
                format!(
                    "Valid behaviors: [{}]",
                    Behavior::behavior_names().join(", ")
                )
            })?;
            let behavior_name = *brain
                .menu()
                .first()
                .expect("Brains should have at least one behavior name");
            return Ok(Self {
                brain: Brain::from_str(s, false)?,
                behavior_name,
                behavior_index: 0,
            });
        };

        let brain = Brain::from_str(brain_name, true)?;

        let (behavior_index, behavior_name) = brain
            .menu()
            .into_iter()
            .enumerate()
            .find(|(_, name)| **name == behavior_name)
            .map(|(i, n)| (i, *n))
            .ok_or_else(|| {
                format!(
                    "Valid behaviors: [{}]",
                    Behavior::behavior_names().join(", ")
                )
            })?;

        Ok(Self {
            brain,
            behavior_name,
            behavior_index,
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
