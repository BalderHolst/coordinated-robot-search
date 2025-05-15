use botbrain::{Map, MapCell, RobotId, RobotPose, messaging::MessageKind, utils::CoverageGrid};
use futures::StreamExt;
use std::{path::PathBuf, rc::Rc, sync::Arc, time::Duration};
use tokio::{
    sync::{Mutex, RwLock},
    task,
};

use arrow_array::{Array, Float32Array, Float64Array, RecordBatch};
use arrow_schema::{DataType, Field, Schema, SchemaBuilder};

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

fn save_batch(batch: &mut RecordBatch, path: &PathBuf) -> Result<(), String> {
    let file = || {
        _ = path.parent().map(|p| std::fs::create_dir_all(p));
        std::fs::File::create(path)
            .map_err(|e| format!("Failed to create file '{}': {}", path.display(), e))
    };

    match path.extension().and_then(|ext| ext.to_str()) {
        Some("ipc") => {
            let mut file = file()?;
            let mut writer = arrow_ipc::writer::FileWriter::try_new(&mut file, &batch.schema())
                .map_err(|e| {
                    format!(
                        "Failed to create IPC writer for file '{}': {}",
                        path.display(),
                        e
                    )
                })?;
            writer.write(batch).map_err(|e| {
                format!("Failed to write batch to file '{}': {}", path.display(), e)
            })?;
            writer.finish().map_err(|e| {
                format!(
                    "Failed to finish writing batch to file '{}': {}",
                    path.display(),
                    e
                )
            })
        }
        Some(other) => Err(format!("Unknown extension: '{}'", other)),
        None => Err(format!(
            "File '{}' has no extension. Could not determine what file to write.",
            path.display()
        )),
    }
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

async fn get_ros2_time(node: &Mutex<Node>) -> Duration {
    node.lock()
        .await
        .get_ros_clock()
        .lock()
        .unwrap()
        .get_now()
        .unwrap()
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
    timeout: Option<Duration>,
    output_path: PathBuf,
    robot_count: usize,
    robot_positions: Rc<RwLock<PositionTable>>,
    time_data: Rc<RwLock<Vec<f32>>>,
    coverage_data: Rc<RwLock<Vec<f32>>>,
    robot_data: Rc<RwLock<Vec<RobotData>>>,
}

impl DataLogger {
    fn new() -> Self {
        let ctx = r2r::Context::create().unwrap();
        let mut node = r2r::Node::create(ctx, "data_logger", "").unwrap();
        let logger = node.logger().to_string().leak();

        let topic: String = node.get_parameter("topic").unwrap_or(DEFAULT_TOPIC.into());

        let robot_count: usize = node.get_parameter::<i64>("robot_count").unwrap_or(1) as usize;

        let output = node
            .get_parameter::<String>("output")
            .unwrap_or("ros_data.ipc".into());
        let output_path = PathBuf::from(output);

        let timeout: f64 = node.get_parameter::<f64>("timeout").unwrap_or({
            node.get_parameter::<i64>("timeout")
                .map_or(0.0, |t| t.max(0) as f64)
        });

        let timeout = match timeout > 0.0 {
            true => Some(Duration::from_secs_f64(timeout)),
            false => None,
        };

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
            timeout,
            output_path,
            robot_count,
            robot_positions,
            robot_data,
            time_data: Rc::new(RwLock::new(vec![])),
            coverage_data: Rc::new(RwLock::new(vec![])),
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

    async fn get_ros2_time(&self) -> Duration {
        get_ros2_time(&self.node).await
    }

    async fn wait_for_ros_time(&self) -> Duration {
        let start = self.get_ros2_time().await;
        tokio::time::sleep(Duration::from_millis(300)).await;
        loop {
            let time = self.get_ros2_time().await;
            log_info!(self.logger, "ROS time not ready yet: {:?}", time);
            if start.abs_diff(time) > Duration::from_millis(200) {
                return time;
            } else {
                tokio::time::sleep(Duration::from_millis(1000)).await;
            }
        }
    }

    async fn to_record_batch(&self) -> RecordBatch {
        let mut cols: Vec<Arc<dyn Array>> = vec![];
        let mut big_schema = SchemaBuilder::new();

        big_schema.push(Field::new("time", DataType::Float32, false));
        cols.push(Arc::new(Float32Array::from(
            self.time_data.read().await.clone(),
        )));

        big_schema.push(Field::new("coverage", DataType::Float32, false));
        cols.push(Arc::new(Float32Array::from(
            self.coverage_data.read().await.clone(),
        )));

        // Add robot data
        for robot in self.robot_data.read().await.iter().cloned() {
            let robot_batch = robot.into_batch();
            let robot_schema = robot_batch.schema();
            for field in robot_schema.fields() {
                big_schema.push(field.clone());
            }
            for col in robot_batch.columns() {
                cols.push(col.clone());
            }
        }

        let big_schema = Arc::new(big_schema.finish());

        RecordBatch::try_new(big_schema, cols).expect("Failed to create RecordBatch")
    }

    async fn run(&self) {
        match self.timeout {
            Some(timeout) => {
                log_info!(self.logger, "Timeout set to {}s", timeout.as_secs_f32());
            }
            None => {
                log_info!(self.logger, "No timeout set");
            }
        }

        {
            let node = self.node.clone();
            let logger = self.logger.to_string();
            tokio::task::spawn_local(async move {
                log_info!(&logger, "Node stated");
                loop {
                    {
                        let mut node = node.lock().await;
                        node.spin_once(Duration::ZERO);
                    }
                    tokio::time::sleep(Duration::from_secs_f32(1.0 / NODE_UPDATE_HZ)).await;
                }
            });
        }

        let Some(map) = self.get_map().await else {
            log_error!(self.logger, "Could not get map :(");
            return;
        };
        log_info!(self.logger, "Map recieved");

        {
            let mut node = self.node.lock().await;
            if node.get_parameter::<bool>("use_sim_time").unwrap_or(true) {
                log_info!(self.logger, "Waiting for simulated ROS time...");
                node.get_time_source()
                    .enable_sim_time(&mut node)
                    .expect("Could not use sim time");
            } else {
                log_info!(self.logger, "Waiting for real ROS time...");
            }
        }
        let start_time = self.wait_for_ros_time().await;
        log_info!(self.logger, "ROS time is ready!");

        // Utility for calculating map coverage
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

        // Logger
        {
            let logger = self.logger.to_string().leak();
            let robot_positions = self.robot_positions.clone();
            let robot_count = self.robot_count;
            let robot_data = self.robot_data.clone();
            let time_data = self.time_data.clone();
            let coverage_data = self.coverage_data.clone();
            let coverage_grid = coverage_grid.clone();
            let node = self.node.clone();
            let timeout = self.timeout;
            tokio::task::spawn_local(async move {
                loop {
                    {
                        let robot_positions = robot_positions.read().await;
                        let coverage_grid = coverage_grid.read().await;
                        let coverage = coverage_grid.coverage();
                        let time = get_ros2_time(&node).await.abs_diff(start_time);
                        log_info!(
                            logger,
                            "time: {:?}s, Coverage {}%, Robot positions: {:?}",
                            time.as_secs_f32(),
                            coverage * 100.0,
                            robot_positions.iter().map(|p| p.pos).collect::<Vec<_>>()
                        );

                        let mut robot_data = robot_data.write().await;
                        for i in 0..robot_count {
                            let pose = &robot_positions[i];
                            robot_data[i].push_state(pose);
                        }
                        {
                            time_data.write().await.push(time.as_secs_f32());
                            coverage_data.write().await.push(coverage);
                        }

                        if let Some(timeout) = timeout {
                            if time > timeout {
                                log_info!(logger, "Timeout reached, shutting down...");
                                break;
                            }
                        }
                    }

                    tokio::time::sleep(Duration::from_secs_f32(1.0 / LOG_HZ)).await;
                }
            })
            .await;
        }

        let batch = self.to_record_batch().await;

        log_info!(self.logger, "Saving data to '{}'", self.output_path.display());

        save_batch(&mut batch.clone(), &self.output_path).map_err(|e| {
            log_error!(self.logger, "Failed to save batch: {}", e);
        });

        log_info!(self.logger, "SHUTTING DOWN!");
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

    local.run_until(data_logger.run()).await;

    Ok(())
}
