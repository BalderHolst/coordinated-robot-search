use clap::{self, Parser};

#[derive(Parser)]
pub struct Args {
    #[arg(index = 1)]
    pub behavior: Behavior,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum Behavior {
    Circle,
    AvoidObstacles,
}

impl Behavior {
    pub fn get_fn(&self) -> fn(&mut dyn robcore::Robot) {
        match self {
            Behavior::Circle => robcore::behaviors::circle,
            Behavior::AvoidObstacles => robcore::behaviors::avoid_obstacles,
        }
    }
}
