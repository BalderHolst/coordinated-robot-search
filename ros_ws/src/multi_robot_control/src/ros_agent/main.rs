mod agent;
mod camera_info;
mod convert_msg;
mod vision;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut agent = agent::RosAgent::new();
    agent.run()
}
