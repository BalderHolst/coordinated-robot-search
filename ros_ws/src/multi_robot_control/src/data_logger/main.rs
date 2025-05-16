use botbrain::{Map, MapCell, Pos2, RobotPose, Vec2, utils::CoverageGrid};
use futures::StreamExt;
use std::{path::PathBuf, rc::Rc, sync::Arc, time::Duration};
use tokio::sync::{Mutex, RwLock};

use arrow_array::{Array, Float32Array, RecordBatch};
use arrow_schema::{DataType, Field, Schema, SchemaBuilder};

const MAP_TOPIC: &str = "/map";

const LOG_HZ: f32 = 5.0;
const NODE_UPDATE_HZ: f32 = 10.0;

use r2r::{
    self, Node, QosProfile,
    geometry_msgs::msg::{PoseWithCovarianceStamped, Quaternion},
    log_error, log_info,
    nav_msgs::msg::OccupancyGrid,
};

type PoseTable = Vec<RobotPose>;

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
    timeout: Option<Duration>,
    output_path: PathBuf,
    robot_count: usize,
    robot_poses: Rc<RwLock<PoseTable>>,
    time_data: Rc<RwLock<Vec<f32>>>,
    coverage_data: Rc<RwLock<Vec<f32>>>,
    robot_data: Rc<RwLock<Vec<RobotData>>>,
}

impl DataLogger {
    fn new() -> Self {
        let ctx = r2r::Context::create().unwrap();
        let node = r2r::Node::create(ctx, "data_logger", "").unwrap();
        let logger = node.logger().to_string().leak();

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

        let robot_poses = Rc::new(RwLock::new(vec![Default::default(); robot_count]));

        let node = Rc::new(Mutex::new(node));

        Self {
            node,
            logger,
            timeout,
            output_path,
            robot_count,
            robot_poses,
            robot_data,
            time_data: Rc::new(RwLock::new(vec![])),
            coverage_data: Rc::new(RwLock::new(vec![])),
        }
    }

    /// Returns the `botbrain` map and the origin of the ROS map
    async fn get_map(&self) -> Option<(Map, Vec2)> {
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
        sub_map.next().await.map(|grid| {
            let origin = Vec2 {
                x: grid.info.origin.position.x as f32,
                y: grid.info.origin.position.y as f32,
            };
            let map = ros2_map_to_botbrain_map(grid);
            (map, origin)
        })
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
            _ = tokio::task::spawn_local(async move {
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

        let Some((map, origin)) = self.get_map().await else {
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

        let map_origin = Rc::new(origin);

        // Utility for calculating map coverage
        let coverage_grid = Rc::new(RwLock::new(CoverageGrid::new(map)));

        let mut robot_poses_ready = Rc::new(Mutex::new(vec![false; self.robot_count]));
        let poses_ready = Rc::new(RwLock::new(false));

        // Spawn message listener
        {
            for id in 0..self.robot_count {
                let qos = QosProfile::default().transient_local();
                let logger = self.logger.to_string();
                let robot_poses = self.robot_poses.clone();
                let robot_poses_ready = robot_poses_ready.clone();
                let poses_ready = poses_ready.clone();
                let coverage_grid = coverage_grid.clone();
                let map_origin = map_origin.clone();

                let topic = format!("/robot_{id}/amcl_pose");
                let mut sub = {
                    let mut node = self.node.lock().await;
                    node.subscribe::<PoseWithCovarianceStamped>(&topic, qos)
                        .unwrap()
                };

                _ = tokio::task::spawn_local(async move {
                    r2r::log_info!(&logger, "Logging data from topic: '{}'", topic);
                    while let Some(msg) = sub.next().await {
                        let mut robot_poses = robot_poses.write().await;
                        let mut coverage_grid = coverage_grid.write().await;

                        let poses_ready_val = { *poses_ready.read().await };

                        if !poses_ready_val {
                            let mut robot_poses_ready = robot_poses_ready.lock().await;
                            robot_poses_ready[id] = true;

                            let mut poses_ready = poses_ready.write().await;
                            *poses_ready = robot_poses_ready.iter().all(|b| *b);
                            log_info!(&logger, "Robots ready {:?}", robot_poses_ready);
                        }

                        let map_size = coverage_grid.map().size();

                        let pos = &msg.pose.pose.position;
                        let pos: Pos2 = Pos2::new(pos.x as f32, pos.y as f32);
                        let pos =
                            Pos2::new(pos.x as f32, pos.y as f32) - *map_origin - map_size / 2.0;

                        let Quaternion { x, y, z, w } = &msg.pose.pose.orientation;
                        let angle =
                            f64::atan2(2. * (w * z + x * y), 1. - 2. * (y.powi(2) + z.powi(2)));

                        Self::set_robot_pose(
                            &mut robot_poses,
                            &mut coverage_grid,
                            id,
                            RobotPose {
                                pos,
                                angle: angle as f32,
                            },
                        );
                    }
                });
            }
        }

        // Logger
        {
            let logger = self.logger.to_string().leak();
            let robot_poses = self.robot_poses.clone();
            let robot_count = self.robot_count;
            let robot_data = self.robot_data.clone();
            let time_data = self.time_data.clone();
            let coverage_data = self.coverage_data.clone();
            let coverage_grid = coverage_grid.clone();
            let node = self.node.clone();
            let timeout = self.timeout;
            let poses_ready = poses_ready.clone();
            _ = tokio::task::spawn_local(async move {
                println!("Starting logger for {} robots", robot_count);
                loop {
                    {
                        let poses_ready = { *poses_ready.read().await };

                        if !poses_ready {
                            log_info!(&logger, "Waiting for robot positions...");
                            tokio::time::sleep(Duration::from_secs_f32(1.0 / LOG_HZ)).await;
                            continue;
                        }

                        let robot_poses = robot_poses.read().await;
                        let coverage_grid = coverage_grid.read().await;
                        let coverage = coverage_grid.coverage();
                        let time = get_ros2_time(&node).await.abs_diff(start_time);
                        log_info!(
                            logger,
                            "time: {:?}s, Coverage {}%, Robots: {}",
                            time.as_secs_f32(),
                            coverage * 100.0,
                            robot_poses
                                .iter()
                                .map(|p| format!(
                                    "(x: {:.3}, y: {:.3}, a: {:.3})",
                                    p.pos.x, p.pos.y, p.angle
                                ))
                                .collect::<Vec<String>>()
                                .join(", ")
                        );

                        let mut robot_data = robot_data.write().await;
                        for i in 0..robot_count {
                            let pose = &robot_poses[i];
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

        log_info!(
            self.logger,
            "Saving data to '{}'",
            self.output_path.display()
        );

        _ = save_batch(&mut batch.clone(), &self.output_path).map_err(|e| {
            log_error!(self.logger, "Failed to save batch: {}", e);
        });

        log_info!(self.logger, "SHUTTING DOWN!");
    }

    fn set_robot_pose(
        robot_poses: &mut PoseTable,
        coverage_grid: &mut CoverageGrid,
        robot_id: usize,
        pose: RobotPose,
    ) {
        robot_poses[robot_id] = pose.clone();
        coverage_grid.mark_pose(pose);
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let local = tokio::task::LocalSet::new();

    let data_logger = DataLogger::new();

    local.run_until(data_logger.run()).await;

    Ok(())
}
