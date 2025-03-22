use agent::RosAgent;

mod agent;
mod camera;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut agent = RosAgent::new();
    agent.run()
}
