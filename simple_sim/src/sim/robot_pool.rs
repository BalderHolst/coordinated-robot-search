use std::{
    collections::{BTreeMap, HashMap},
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc,
    },
    thread,
};

use botbrain::{
    behaviors::{BehaviorFn, CreateFn},
    Robot, RobotId,
};

use crate::world::{self, World};

use super::{step::step_agent, RobotState, StepArgs, StepDiagnostic};

type WorkerInput = (RobotId, RobotState, Arc<StepArgs>);
type WorkerOutput = (RobotId, StepDiagnostic, RobotState);

pub struct RobotThreadPool {
    threads: usize,
    robot_map: HashMap<RobotId, usize>,
    input_channels: Vec<Sender<Option<WorkerInput>>>,
    output_channel: Receiver<WorkerOutput>,
    handles: Vec<thread::JoinHandle<()>>,
    next_id: u32,
}

struct ThreadCtx {
    robots: Vec<Box<dyn Robot + 'static>>,
    behavior_fn: BehaviorFn,
    create_robot_fn: CreateFn,
    world: World,
}

impl ThreadCtx {
    fn create_robot(&self, id: RobotId) -> Box<dyn Robot> {
        let mut robot = (self.create_robot_fn)();
        robot.set_id(id);
        robot.set_map(world::convert_to_botbrain_map(&self.world));
        robot.get_debug_soup_mut().activate();
        robot
    }
}

impl RobotThreadPool {
    pub fn new(
        threads: usize,
        behavior_fn: BehaviorFn,
        create_robot_fn: CreateFn,
        world: World,
    ) -> RobotThreadPool {
        let (output_tx, output_rx) = mpsc::channel();

        let (input_channels, handles): (Vec<_>, Vec<_>) = (0..threads)
            .map(|thread_id| {
                let (input_tx, input_rx) =
                    mpsc::channel::<Option<WorkerInput>>();
                let output_tx = output_tx.clone();
                let world = world.clone();
                let handle = {
                    thread::spawn(move || {
                        let mut ctx = ThreadCtx {
                            robots: vec![],
                            behavior_fn,
                            create_robot_fn,
                            world,
                        };
                        while let Ok(Some((id, mut state, args))) = input_rx.recv() {
                            let robot =
                                match ctx.robots.iter_mut().find(|robot| *robot.get_id() == id) {
                                    Some(robot) => robot,
                                    None => {
                                        println!(
                                            "[INFO] [thread {thread_id}] Creating new robot with id {id:?}!"
                                        );
                                        let robot = ctx.create_robot(id);
                                        ctx.robots.push(robot);
                                        ctx.robots.last_mut().unwrap()
                                    }
                                };

                            let start = std::time::Instant::now();
                            step_agent(&mut state, robot, &args, ctx.behavior_fn);
                            let step_time = start.elapsed().as_secs_f32();

                            output_tx.send((id, StepDiagnostic { step_time }, state)).unwrap();
                        }
                    })
                };
                (input_tx, handle)
            })
            .unzip();

        RobotThreadPool {
            threads,
            input_channels,
            output_channel: output_rx,
            handles,
            robot_map: HashMap::new(),
            next_id: 0,
        }
    }

    pub fn add_robot(&mut self, state: RobotState, args: Arc<StepArgs>) {
        let id = RobotId::new(self.next_id);
        self.next_id += 1;

        let thread_index = id.as_u32() as usize % self.threads;

        let channel = &self.input_channels[thread_index];
        channel.send(Some((id, state, args))).unwrap();

        // We dont care about the output
        self.output_channel.recv().unwrap();

        self.robot_map.insert(id, thread_index);
    }

    pub fn process(
        &self,
        inputs: Vec<(RobotId, RobotState)>,
        args: Arc<StepArgs>,
    ) -> Vec<WorkerOutput> {
        let len = inputs.len();

        for (id, state) in inputs {
            let thread_index = self.robot_map.get(&id).unwrap();
            self.input_channels[*thread_index]
                .send(Some((id, state, args.clone())))
                .unwrap();
        }

        let mut results = BTreeMap::new();
        for _ in 0..len {
            let (id, diag, state) = self.output_channel.recv().unwrap();
            results.insert(id, (diag, state));
        }
        results
            .into_iter()
            .map(|(id, (diag, state))| (id, diag, state))
            .collect()
    }
}

impl Drop for RobotThreadPool {
    fn drop(&mut self) {
        for input_channel in self.input_channels.drain(..) {
            // Send None to signal the thread to stop their internal loop
            input_channel.send(None).unwrap();
        }
        for handle in self.handles.drain(..) {
            // Wait for all threads to finish
            handle.join().unwrap();
        }
    }
}
