use botbrain::{Map, MapCell, RobotId, RobotPose, messaging::MessageKind, utils::CoverageGrid};
use futures::StreamExt;
use std::{rc::Rc, sync::Arc, time::Duration};
use tokio::{
    sync::{Mutex, RwLock},
    task,
};

use arrow_array::{Float32Array, RecordBatch};
use arrow_schema::{DataType, Field, Schema};

const MAP_TOPIC: &str = "/map";
const DEFAULT_TOPIC: &str = "/search_channel";

const LOG_HZ: f32 = 5.0;
const NODE_UPDATE_HZ: f32 = 10.0;

use r2r::{
    self, Node, QosProfile, log_error, log_info, nav_msgs::msg::OccupancyGrid,
    ros_agent_msgs::msg::AgentMessage,
};

type PositionTable = Vec<RobotPose>;

#[derive(Clone, Default)]
struct RobotData {
    prefix: String,
    x: Vec<f32>,
    y: Vec<f32>,
    angle: Vec<f32>,
}

pub fn ros2_map_to_botbrain_map(map: OccupancyGrid) -> Map {
    let (width, height) = (map.info.width, map.info.height);
    let scale = map.info.resolution;
    let mut botbrain_map = Map::new(width as f32 * scale, height as f32 * scale, scale);

    for y in 0..height {
        for x in 0..width {
            let cell = map.data[y as usize * width as usize + x as usize];
            let cell = match cell {
                100 => MapCell::Obstacle,
                _ => MapCell::Free,
            };
            botbrain_map.grid_mut().set(x as usize, y as usize, cell);
        }
    }

    botbrain_map
}

impl RobotData {
    fn new(capacity: usize, prefix: &str) -> Self {
        let prefix = match prefix.is_empty() {
            true => "".to_string(),
            false => format!("{prefix}/"),
        };

        Self {
            prefix,
            x: Vec::with_capacity(capacity),
            y: Vec::with_capacity(capacity),
            angle: Vec::with_capacity(capacity),
        }
    }

    fn schema(&self) -> Schema {
        let prefix = &self.prefix;
        Schema::new(vec![
            Field::new(format!("{prefix}x"), DataType::Float32, false),
            Field::new(format!("{prefix}y"), DataType::Float32, false),
            Field::new(format!("{prefix}angle"), DataType::Float32, false),
        ])
    }

    fn push_state(&mut self, pose: &RobotPose) {
        self.x.push(pose.pos.x);
        self.y.push(pose.pos.y);
        self.angle.push(pose.angle);
    }

    fn into_batch(self) -> RecordBatch {
        let schema = Arc::new(self.schema());
        RecordBatch::try_new(
            schema,
            vec![
                Arc::<Float32Array>::new(self.x.into()),
                Arc::<Float32Array>::new(self.y.into()),
                Arc::<Float32Array>::new(self.angle.into()),
            ],
        )
        .expect("Failed to create RecordBatch")
    }
}

struct DataLogger {
    node: Rc<Mutex<Node>>,
    logger: &'static str,
    topic: String,
    robot_count: usize,
    robot_positions: Rc<RwLock<PositionTable>>,
    robot_data: Rc<RwLock<Vec<RobotData>>>,
}

impl DataLogger {
    fn new() -> Self {
        let ctx = r2r::Context::create().unwrap();
        let mut node = r2r::Node::create(ctx, "data_logger", "").unwrap();
        let logger = node.logger().to_string().leak();

        let topic: String = node.get_parameter("topic").unwrap_or(DEFAULT_TOPIC.into());

        let robot_count: usize = node.get_parameter::<i64>("robot_count").unwrap_or(1) as usize;

        let robot_data = Rc::new(RwLock::new(
            (0..robot_count)
                .map(|i| RobotData::new(robot_count, &(format!("robot_{i}"))))
                .collect::<Vec<_>>(),
        ));

        let robot_positions = Rc::new(RwLock::new(vec![RobotPose::default(); robot_count]));

        let node = Rc::new(Mutex::new(node));

        Self {
            node,
            logger,
            topic,
            robot_count,
            robot_positions,
            robot_data,
        }
    }

    async fn get_map(&self) -> Option<Map> {
        let mut sub_map;
        {
            let mut node = self.node.lock().await;
            sub_map = node
                .subscribe::<OccupancyGrid>(MAP_TOPIC, QosProfile::default().transient_local())
                .unwrap();
        }

        let logger = self.logger.to_string();

        // We only need the map once
        log_info!(&logger, "Listening for map on '{MAP_TOPIC}'...");
        sub_map.next().await.map(ros2_map_to_botbrain_map)
    }

    async fn run(&self) {
        let node = self.node.clone();
        tokio::task::spawn_local(async move {
            println!("Node stated");
            loop {
                {
                    let mut node = node.lock().await;
                    println!("Node running");
                    node.spin_once(Duration::ZERO);
                }
                tokio::time::sleep(Duration::from_secs_f32(1.0 / NODE_UPDATE_HZ)).await;
            }
        });

        let Some(map) = self.get_map().await else {
            log_error!(self.logger, "Could not get map :(");
            return;
        };

        log_info!(self.logger, "Map recieved");

        let coverage_grid = Rc::new(RwLock::new(CoverageGrid::new(map)));

        // Spawn message listener
        {
            let qos = QosProfile::default().volatile();
            let mut node = self.node.lock().await;
            let mut sub = node.subscribe::<AgentMessage>(&self.topic, qos).unwrap();

            let logger = self.logger.to_string().leak();
            let topic = self.topic.to_string().leak();
            let robot_positions = self.robot_positions.clone();
            let coverage_grid = coverage_grid.clone();
            tokio::task::spawn_local(async move {
                r2r::log_info!(logger, "Logging data from topic: '{}'", topic);
                while let Some(msg) = sub.next().await {
                    (async |msg: AgentMessage| match MessageKind::try_from(msg.data) {
                        Ok(kind) => {
                            if let Some(pose) = kind.pose() {
                                let id = RobotId::new(msg.sender_id);
                                {
                                    let mut coverage_grid = coverage_grid.write().await;
                                    let mut robot_positions = robot_positions.write().await;
                                    Self::set_robot_pose(
                                        &mut robot_positions,
                                        &mut coverage_grid,
                                        id,
                                        pose,
                                    );
                                }
                            }
                        }
                        Err(e) => {
                            log_error!(logger, "[{}] Corrupt message: {}", msg.sender_id, e)
                        }
                    })(msg)
                    .await;
                }
            });
        }
        // Spawn data logger

        {
            let robot_data = self.robot_data.clone();
            let robot_positions = self.robot_positions.clone();
            let robot_count = self.robot_count;
            tokio::task::spawn_local(async move {
                loop {
                    {
                        let mut robot_data = robot_data.write().await;
                        let robot_positions = robot_positions.read().await;
                        for i in 0..robot_count {
                            let pose = &robot_positions[i];
                            robot_data[i].push_state(pose);
                        }
                    }
                    tokio::time::sleep(Duration::from_secs_f32(1.0 / LOG_HZ)).await;
                }
            });
        }

        // Printer
        {
            let logger = self.logger.to_string().leak();
            let robot_positions = self.robot_positions.clone();
            let coverage_grid = coverage_grid.clone();
            tokio::task::spawn_local(async move {
                loop {
                    {
                        let robot_positions = robot_positions.read().await;
                        let coverage_grid = coverage_grid.read().await;
                        log_info!(
                            logger,
                            "Coverage {}%, Robot positions: {:?}",
                            coverage_grid.coverage() * 100.0,
                            robot_positions.iter().map(|p| p.pos).collect::<Vec<_>>()
                        );
                    }
                    tokio::time::sleep(Duration::from_secs(1)).await;
                }
            });
        }
    }

    fn set_robot_pose(
        robot_positions: &mut PositionTable,
        coverage_grid: &mut CoverageGrid,
        robot_id: RobotId,
        pose: RobotPose,
    ) {
        robot_positions[robot_id.as_u32() as usize] = pose.clone();
        coverage_grid.mark_pose(pose);
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let local = tokio::task::LocalSet::new();

    let mut data_logger = DataLogger::new();

    local
        .run_until(async {
            data_logger.run().await;
            futures::future::pending::<()>().await;
        })
        .await;

    Ok(())
}
