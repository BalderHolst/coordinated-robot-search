use agent::RosAgent;

mod agent;
mod camera_info;
mod vision;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut agent = RosAgent::new();
    agent.run()
}
