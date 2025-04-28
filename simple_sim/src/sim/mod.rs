mod collisions;
#[cfg(not(feature = "single-thread"))]
mod robot_pool;
mod step;

use std::{
    sync::{mpsc, Arc},
    time::Instant,
};

use arrow_array::{Array, Float32Array, Float64Array, Int16Array, RecordBatch};
use arrow_schema::{DataType, Field, Schema, SchemaBuilder};
use botbrain::{
    self,
    behaviors::{Behavior, Time},
    debug::{DebugSoup, DebugType},
    scaled_grid::ScaledGrid,
    Control, Pos2, RobotId, RobotPose,
};

#[cfg(not(feature = "single-thread"))]
use robot_pool::RobotThreadPool;

use crate::{
    cli::ScenarioArgs,
    scenario::Scenario,
    world::{self, Cell, World},
};

const SPEED_MULTIPLIER: f32 = 1.0;
const STEER_MULTIPLIER: f32 = 1.0;

const LIDAR_RAYS: usize = 40;
const CAMERA_RAYS: usize = 20;

const SIMULATION_DT: f32 = 1.0 / 60.0;

const COVERAGE_GRID_SCALE: f32 = 0.1;

#[derive(Clone, Default)]
struct RobotData {
    prefix: String,
    x: Vec<f32>,
    y: Vec<f32>,
    angle: Vec<f32>,
    vel: Vec<f32>,
    avel: Vec<f32>,
    steer: Vec<f32>,
    speed: Vec<f32>,
    mode: Vec<i16>,
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
            vel: Vec::with_capacity(capacity),
            avel: Vec::with_capacity(capacity),
            steer: Vec::with_capacity(capacity),
            speed: Vec::with_capacity(capacity),
            mode: Vec::with_capacity(capacity),
        }
    }

    fn schema(&self) -> Schema {
        let prefix = &self.prefix;
        Schema::new(vec![
            Field::new(format!("{prefix}x"), DataType::Float32, false),
            Field::new(format!("{prefix}y"), DataType::Float32, false),
            Field::new(format!("{prefix}angle"), DataType::Float32, false),
            Field::new(format!("{prefix}vel"), DataType::Float32, false),
            Field::new(format!("{prefix}avel"), DataType::Float32, false),
            Field::new(format!("{prefix}steer"), DataType::Float32, false),
            Field::new(format!("{prefix}speed"), DataType::Float32, false),
            Field::new(format!("{prefix}mode"), DataType::Int16, false),
        ])
    }

    fn push_state(&mut self, state: &RobotState, control: &Control) {
        self.x.push(state.pose.pos.x);
        self.y.push(state.pose.pos.y);
        self.angle.push(state.pose.angle);
        self.vel.push(state.vel);
        self.avel.push(state.avel);
        self.steer.push(control.steer);
        self.speed.push(control.speed);

        let mode = match state.soup.get("", "mode").map(|t| (*t).clone()) {
            Some(DebugType::Int(mode)) => mode,
            _ => 0,
        };

        self.mode.push(mode as i16);
    }

    fn into_batch(self) -> RecordBatch {
        let schema = Arc::new(self.schema());
        RecordBatch::try_new(
            schema,
            vec![
                Arc::<Float32Array>::new(self.x.into()),
                Arc::<Float32Array>::new(self.y.into()),
                Arc::<Float32Array>::new(self.angle.into()),
                Arc::<Float32Array>::new(self.vel.into()),
                Arc::<Float32Array>::new(self.avel.into()),
                Arc::<Float32Array>::new(self.steer.into()),
                Arc::<Float32Array>::new(self.speed.into()),
                Arc::<Int16Array>::new(self.mode.into()),
            ],
        )
        .expect("Failed to create RecordBatch")
    }
}

pub fn run_scenario_headless(
    mut sim: Simulator,
    scenario: Scenario,
    scenario_args: ScenarioArgs,
) -> RecordBatch {
    let steps = (scenario.duration / SIMULATION_DT).ceil() as usize;

    let mut time_data = Vec::with_capacity(steps);

    let mut robot_data = (0..scenario.robots.len())
        .map(|i| RobotData::new(steps, &format!("robot_{}", i)))
        .collect::<Vec<_>>();

    let mut coverage_data = Vec::with_capacity(steps);

    let start_time = Instant::now();
    let mut last_print = start_time;

    loop {
        if (Instant::now() - last_print).as_secs_f64() > scenario_args.print_interval {
            last_print = Instant::now();
            println!("Time: {:.1}s", sim.state.time.as_secs_f64());
        }

        time_data.push(sim.state.time.as_secs_f64());

        // Calculate and store coverage
        let coverage = sim.state.diagnostics.coverage();
        coverage_data.push(coverage);

        // Store robot data
        for (n, robot_state) in sim.state.robot_states.iter().enumerate() {
            robot_data[n].push_state(robot_state, &robot_state.control);
        }

        if sim.state.time.as_secs_f32() >= scenario.duration {
            break;
        }

        sim.step();
    }

    println!(
        "Simulation finished after {:.2} seconds",
        sim.state.time.as_secs_f32()
    );

    println!();
    println!("Robot poses:");
    for agent in sim.state.robot_states {
        let RobotPose { pos, angle } = agent.pose;
        println!("  pos: {:?}, angle: {:.2}", pos, angle);
    }

    let mut cols: Vec<Arc<dyn Array>> = vec![];

    let mut big_schema = SchemaBuilder::new();

    big_schema.push(Field::new("time", DataType::Float64, false));
    cols.push(Arc::new(Float64Array::from(time_data)));

    big_schema.push(Field::new("coverage", DataType::Float32, false));
    cols.push(Arc::new(Float32Array::from(coverage_data)));

    // Add robot data
    for robot in robot_data {
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

#[derive(Clone, Default)]
pub struct RobotState {
    pub id: RobotId,
    pub pose: RobotPose,
    pub vel: f32,
    pub avel: f32,
    pub control: Control,
    pub soup: DebugSoup,
}

#[derive(Clone)]
pub struct SimState {
    pub robot_states: Vec<RobotState>,
    pub time: Time,
    pub diagnostics: SimDiagnostics,
}

#[derive(Clone)]
pub struct SimDiagnostics {
    pub coverage_grid: ScaledGrid<bool>,
    world: World,
}

impl SimDiagnostics {
    pub fn coverage(&self) -> f32 {
        let mut total_area = 0;
        let covered_cells = self
            .coverage_grid
            .iter()
            .filter(|&(x, y, &covered)| {
                if let Some(&cell) = self.world.get(Pos2 { x, y }) {
                    if cell != Cell::Wall {
                        total_area += 1;
                        covered
                    } else {
                        false
                    }
                } else {
                    false
                }
            })
            .count();
        covered_cells as f32 / total_area as f32
    }
}

#[derive(Clone)]
pub struct SimArgs {
    pub world: World,
    pub behavior: Behavior,
    #[cfg(not(feature = "single-thread"))]
    pub threads: usize,
}

#[allow(clippy::type_complexity)]
pub struct Simulator {
    pub state: SimState,
    pub behavior: Behavior,
    world: World,
    dt: f32,
    msg_send_rx: mpsc::Receiver<botbrain::Message>,
    msg_send_tx: mpsc::Sender<botbrain::Message>,
    pending_msgs: Vec<botbrain::Message>,

    #[cfg(not(feature = "single-thread"))]
    pool: RobotThreadPool,

    #[cfg(feature = "single-thread")]
    robots: Vec<Box<dyn botbrain::Robot>>,
}

impl Simulator {
    pub fn new(args: SimArgs) -> Self {
        let SimArgs {
            world,
            behavior,
            #[cfg(not(feature = "single-thread"))]
            threads,
        } = args;

        let state = SimState {
            robot_states: vec![],
            time: Time::default(),
            diagnostics: SimDiagnostics {
                coverage_grid: ScaledGrid::new(world.width(), world.height(), COVERAGE_GRID_SCALE),
                world: world.clone(),
            },
        };

        let (msg_send_tx, msg_send_rx) = mpsc::channel();

        #[cfg(not(feature = "single-thread"))]
        let pool = RobotThreadPool::new(
            threads,
            behavior.behavior_fn(),
            behavior.create_fn(),
            world.clone(),
        );

        Self {
            state,
            pending_msgs: vec![],
            behavior: behavior.clone(),
            world,
            dt: SIMULATION_DT,
            msg_send_tx,
            msg_send_rx,

            #[cfg(not(feature = "single-thread"))]
            pool,

            #[cfg(feature = "single-thread")]
            robots: vec![],
        }
    }

    pub fn world(&self) -> &World {
        &self.world
    }

    fn create_step_args(&self) -> Arc<StepArgs> {
        Arc::new(StepArgs {
            agents: self.state.robot_states.clone(),
            world: self.world.clone(),
            time: self.state.time,
            dt: self.dt,
            msg_send_tx: self.msg_send_tx.clone(),
            pending_msgs: self.pending_msgs.clone(),
        })
    }

    pub fn add_robot(&mut self, robot_pose: RobotPose) {
        let id = RobotId::new(self.state.robot_states.len() as u32);

        // Add the robot to the state which is managed by the simulator
        let state = RobotState {
            id,
            pose: robot_pose.clone(),
            ..Default::default()
        };
        self.state.robot_states.push(state.clone());

        #[cfg(not(feature = "single-thread"))]
        {
            let args = self.create_step_args();
            self.pool.add_robot(state, args);
        }

        #[cfg(feature = "single-thread")]
        {
            let mut robot = (self.behavior.create_fn())();
            robot.set_id(id);
            robot.set_world(crate::world::convert_to_botbrain_map(&self.world));
            robot.get_debug_soup_mut().activate();
            robot.input_pose(robot_pose);
            self.robots.push(robot);
        }
    }

    pub fn step(&mut self) {
        let args = self.create_step_args();

        #[cfg(not(feature = "single-thread"))]
        {
            // Move the robot states to the robot pool
            let agents = std::mem::replace(&mut self.state.robot_states, Vec::with_capacity(0));

            // Construct the input for the thread pool function
            let input = agents.into_iter().map(|a| (a.id, a)).collect::<Vec<_>>();

            // Process the agents and robots in parallel
            let outputs = self.pool.process(input, args);

            // Move the agents and robots back to the simulator
            self.state.robot_states = outputs.into_iter().map(|(_, s)| s).collect();
        }

        #[cfg(feature = "single-thread")]
        {
            for (state, robot) in self.state.robot_states.iter_mut().zip(&mut self.robots) {
                step::step_agent(state, robot, &args, self.behavior.behavior_fn());
            }
        }

        // Collect all pending messages
        self.pending_msgs = self
            .msg_send_rx
            .try_iter()
            .collect::<Vec<botbrain::Message>>();

        // Update diagnostics
        let diagnostics = &mut self.state.diagnostics;
        for msg in &self.pending_msgs {
            match &msg.kind {
                botbrain::MessageKind::ShapeDiff { shape, diff: _ } => {
                    diagnostics.coverage_grid.set_shape(shape, true)
                }
                botbrain::MessageKind::CamDiff {
                    cone,
                    lidar: _,
                    diff: _,
                } => diagnostics.coverage_grid.set_cone(cone, true),
                botbrain::MessageKind::Debug(_) => {}
            }
        }

        self.state.time += Time::from_secs_f32(self.dt);
    }

    #[allow(unused)]
    pub fn step_duration(&mut self, duration: f32) {
        let steps = (duration / self.dt).ceil() as usize;
        for _ in 0..steps {
            self.step();
        }
    }
}

/// Shared state needed to step an agent forward in time
#[derive(Clone)]
struct StepArgs {
    agents: Vec<RobotState>,
    world: World,
    time: Time,
    dt: f32,
    msg_send_tx: mpsc::Sender<botbrain::Message>,
    pending_msgs: Vec<botbrain::Message>,
}
#[cfg(feature = "single-thread")]
#[allow(unused)]
impl Simulator {
    pub fn with_robots(
        sim_args: SimArgs,
        robots: Vec<(RobotPose, Box<dyn botbrain::Robot>)>,
    ) -> Self {
        let mut sim = Self::new(sim_args);
        for (pose, mut robot) in robots {
            sim.state.robot_states.push(RobotState {
                id: RobotId::new(sim.state.robot_states.len() as u32),
                pose: pose.clone(),
                ..Default::default()
            });

            robot.set_world(world::convert_to_botbrain_map(&sim.world));
            robot.get_debug_soup_mut().activate();
            robot.input_pose(pose);
            sim.robots.push(robot);
        }
        sim
    }

    pub fn robots(&self) -> &[Box<dyn botbrain::Robot>] {
        &self.robots
    }

    pub fn robots_mut(&mut self) -> &mut [Box<dyn botbrain::Robot>] {
        &mut self.robots
    }
}
