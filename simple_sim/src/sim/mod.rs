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
    debug_soup::{DebugItem, DebugSoup},
    messaging::Message,
    utils::CoverageGrid,
    Control, RobotId, RobotPose,
};

#[cfg(not(feature = "single-thread"))]
use robot_pool::RobotThreadPool;

#[cfg(feature = "single-thread")]
use botbrain::Robot;

use crate::{cli::ScenarioArgs, scenario::Scenario, world::World};

const SPEED_MULTIPLIER: f32 = 0.43;
const STEER_MULTIPLIER: f32 = 0.90;

const LIDAR_RAYS: usize = 40;
const CAMERA_RAYS: usize = 20;

const SIMULATION_DT: f32 = 1.0 / 60.0;

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
            Some(DebugItem::Int(mode)) => mode,
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

    let mut step_time_data = Vec::with_capacity(steps);

    let mut prev_msgs = 0;
    let mut msgs_data = Vec::with_capacity(steps);

    let mut prev_bytes = 0;
    let mut msg_bytes_data = Vec::with_capacity(steps);

    let start_time = Instant::now();
    let mut last_print = start_time;

    loop {
        if (Instant::now() - last_print).as_secs_f64() > scenario_args.print_interval {
            last_print = Instant::now();
            println!("Time: {:.1}s", sim.state.time.as_secs_f64());
        }

        time_data.push(sim.state.time.as_secs_f64());

        // Calculate and store coverage
        let coverage = sim.state.diagnostics.coverage_grid.coverage();
        coverage_data.push(coverage);

        // Calculate and store bytes sent
        let new_bytes =
            (sim.state.diagnostics.bytes_sent - prev_bytes) as f64 / SIMULATION_DT as f64;
        prev_bytes = sim.state.diagnostics.bytes_sent;
        msg_bytes_data.push(new_bytes);

        let new_msgs = sim.state.diagnostics.msgs_sent - prev_msgs;
        prev_msgs = sim.state.diagnostics.msgs_sent;
        let new_msgs = new_msgs as f64 / SIMULATION_DT as f64;
        msgs_data.push(new_msgs);

        // Store robot data
        for (n, robot_state) in sim.state.robot_states.iter().enumerate() {
            robot_data[n].push_state(robot_state, &robot_state.control);
        }

        step_time_data.push(sim.state.diagnostics.avg_step_time);

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

    big_schema.push(Field::new("msg-bytes", DataType::Float64, false));
    cols.push(Arc::new(Float64Array::from(msg_bytes_data)));

    big_schema.push(Field::new("msg-count", DataType::Float64, false));
    cols.push(Arc::new(Float64Array::from(msgs_data)));

    big_schema.push(Field::new("step-time", DataType::Float32, false));
    cols.push(Arc::new(Float32Array::from(step_time_data)));

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
    // Total number of bytes sent
    pub msgs_sent: u64,
    pub bytes_sent: u128,
    pub avg_step_time: f32,
    pub coverage_grid: CoverageGrid,
}

#[derive(Clone)]
pub struct SimArgs {
    pub world: World,
    pub behavior: Behavior,
    #[cfg(not(feature = "single-thread"))]
    pub threads: usize,
    pub no_debug_soup: bool,
}

#[allow(clippy::type_complexity)]
pub struct Simulator {
    pub state: SimState,
    pub behavior: Behavior,
    world: World,
    dt: f32,
    msg_send_rx: mpsc::Receiver<Message>,
    msg_send_tx: mpsc::Sender<Message>,
    pending_msgs: Vec<Message>,

    #[cfg(not(feature = "single-thread"))]
    pool: RobotThreadPool,

    #[cfg(feature = "single-thread")]
    robots: Vec<Box<dyn Robot>>,
}

impl Simulator {
    pub fn new(args: SimArgs) -> Self {
        let SimArgs {
            world,
            behavior,
            #[cfg(not(feature = "single-thread"))]
            threads,
            no_debug_soup,
        } = args;

        let state = SimState {
            robot_states: vec![],
            time: Time::default(),
            diagnostics: SimDiagnostics {
                coverage_grid: CoverageGrid::new(crate::world::convert_to_botbrain_map(&world)),
                msgs_sent: 0,
                bytes_sent: 0,
                avg_step_time: 0.0,
            },
        };

        let (msg_send_tx, msg_send_rx) = mpsc::channel();

        #[cfg(not(feature = "single-thread"))]
        let pool = RobotThreadPool::new(
            threads,
            behavior.behavior_fn(),
            behavior.create_fn(),
            world.clone(),
            no_debug_soup,
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
            robot.set_map(crate::world::convert_to_botbrain_map(&self.world));
            // FIX: Remove activation here in single-thread mode
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
            let len = self.state.robot_states.len();
            let agents = std::mem::replace(&mut self.state.robot_states, Vec::with_capacity(len));

            // Construct the input for the thread pool function
            let input = agents.into_iter().map(|a| (a.id, a)).collect::<Vec<_>>();

            // Process the agents and robots in parallel
            let outputs = self.pool.process(input, args);

            let mut avg_step_time = 0.0;
            for (_id, diag, state) in outputs {
                // Update the robot state
                self.state.robot_states.push(state);

                // Update the diagnostics
                avg_step_time += diag.step_time;
            }
            self.state.diagnostics.avg_step_time =
                avg_step_time / self.state.robot_states.len() as f32;
        }

        #[cfg(feature = "single-thread")]
        {
            for (state, robot) in self.state.robot_states.iter_mut().zip(&mut self.robots) {
                step::step_agent(state, robot, &args, self.behavior.behavior_fn());
            }
        }

        // Collect all pending messages
        self.pending_msgs = self.msg_send_rx.try_iter().collect::<Vec<Message>>();

        let diagnostics = &mut self.state.diagnostics;

        // Update coverage grid
        for pose in self.state.robot_states.iter().map(|r| r.pose.clone()) {
            diagnostics.coverage_grid.mark_pose(pose);
        }

        // Update total bytes sent
        for msg in &self.pending_msgs {
            self.state.diagnostics.msgs_sent += 1;
            if let Ok(bytes) = msg.encode() {
                let bytes_sent = &mut self.state.diagnostics.bytes_sent;
                *bytes_sent = bytes_sent.wrapping_add(bytes.len() as u128);
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
    msg_send_tx: mpsc::Sender<Message>,
    pending_msgs: Vec<Message>,
}
#[cfg(feature = "single-thread")]
#[allow(unused)]
impl Simulator {
    pub fn with_robots(sim_args: SimArgs, robots: Vec<(RobotPose, Box<dyn Robot>)>) -> Self {
        let no_debug_soup = sim_args.no_debug_soup;
        let mut sim = Self::new(sim_args);
        for (pose, mut robot) in robots {
            sim.state.robot_states.push(RobotState {
                id: RobotId::new(sim.state.robot_states.len() as u32),
                pose: pose.clone(),
                ..Default::default()
            });

            robot.set_map(crate::world::convert_to_botbrain_map(&sim.world));
            if !no_debug_soup {
                robot.get_debug_soup_mut().activate();
            }
            robot.input_pose(pose);
            sim.robots.push(robot);
        }
        sim
    }

    pub fn robots(&self) -> &[Box<dyn Robot>] {
        &self.robots
    }

    pub fn robots_mut(&mut self) -> &mut [Box<dyn Robot>] {
        &mut self.robots
    }
}

#[derive(Clone, Default)]
pub struct StepDiagnostic {
    #[allow(unused)]
    pub step_time: f32,
}
