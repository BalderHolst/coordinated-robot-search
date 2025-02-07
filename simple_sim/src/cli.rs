use clap::{self, Parser};

#[derive(Parser)]
pub struct Args {
    #[arg(index = 1)]
    pub behavior: Behavior,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum Behavior {
    Nothing,
    Circle,
    OnlyStraight,
    AvoidObstacles,
    TowardSpace,
}

impl Behavior {
    pub fn get_fn(&self) -> fn(&mut dyn robcore::Robot) {
        match self {
            Self::Nothing => robcore::behaviors::nothing,
            Self::Circle => robcore::behaviors::circle,
            Self::OnlyStraight => robcore::behaviors::only_straight,
            Self::AvoidObstacles => robcore::behaviors::avoid_obstacles,
            Self::TowardSpace => robcore::behaviors::toward_space,
        }
    }
}
