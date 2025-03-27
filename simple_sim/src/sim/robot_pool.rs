use std::{
    collections::{BTreeMap, HashMap},
    f32::consts::PI,
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc, Mutex,
    },
    thread,
};

use botbrain::{
    behaviors::{BehaviorFn, CreateFn},
    debug::DebugSoup,
    params::{CAM_FOV, CAM_RANGE, DIAMETER, LIDAR_RANGE},
    CamData, CamPoint, LidarData, Message, Robot, RobotId, RobotPose, Vec2,
};

use crate::world::Cell;

use super::{
    cast_ray, collisions, RobotState, StepArgs, CAMERA_RAYS, LIDAR_RAYS, SPEED_MULTIPLIER,
    STEER_MULTIPLIER,
};

pub struct RobotInput {
    pose: Option<RobotPose>,
    cam: Option<CamData>,
    lidar: Option<LidarData>,
    msgs: Option<Vec<Message>>,
}

impl RobotInput {
    fn empty() -> Self {
        Self {
            pose: None,
            cam: None,
            lidar: None,
            msgs: None,
        }
    }

    fn update_robot(self, robot: &mut Box<dyn Robot>) {
        if let Some(pose) = self.pose {
            robot.input_pose(pose);
        }
        if let Some(cam) = self.cam {
            robot.input_cam(cam);
        }
        if let Some(lidar) = self.lidar {
            robot.input_lidar(lidar);
        }
        if let Some(msgs) = self.msgs {
            robot.input_msgs(msgs);
        }
    }
}

pub struct RobotThreadPool {
    threads: usize,
    robot_map: HashMap<RobotId, usize>,
    soups: HashMap<RobotId, Arc<Mutex<DebugSoup>>>,
    input_channels: Vec<Sender<Option<(RobotId, RobotState, Arc<StepArgs>)>>>,
    output_channel: Receiver<(RobotId, RobotState)>,
    handles: Vec<thread::JoinHandle<()>>,
    next_id: u32,
}

struct ThreadCtx {
    robots: Vec<Box<dyn Robot + 'static>>,
    behavior_fn: BehaviorFn,
    create_robot_fn: CreateFn,
    world_size: Vec2,
}

impl ThreadCtx {
    fn create_robot(&self, id: RobotId) -> Box<dyn Robot> {
        let mut robot = (self.create_robot_fn)();
        robot.set_id(id);
        robot.set_world_size(self.world_size);
        robot.get_debug_soup_mut().activate();
        robot
    }
}

impl RobotThreadPool {
    pub fn new(
        threads: usize,
        behavior_fn: BehaviorFn,
        create_robot_fn: CreateFn,
        world_size: Vec2,
    ) -> RobotThreadPool {
        let (output_tx, output_rx) = mpsc::channel();

        let ctxs = (0..threads).map(|_| ThreadCtx {
            robots: vec![],
            behavior_fn,
            create_robot_fn,
            world_size,
        });

        let (input_channels, handles): (Vec<_>, Vec<_>) = ctxs
            .into_iter()
            .enumerate()
            .map(|(thread_id, mut ctx)| {
                let (input_tx, input_rx) =
                    mpsc::channel::<Option<(RobotId, RobotState, Arc<StepArgs>)>>();
                let output_tx = output_tx.clone();
                let handle = {
                    thread::spawn(move || {
                        while let Ok(Some((id, mut state, args))) = input_rx.recv() {
                            let robot =
                                match ctx.robots.iter_mut().find(|robot| *robot.get_id() == id) {
                                    Some(robot) => robot,
                                    None => {
                                        println!(
                                            "[{thread_id}] Creating new robot with id {id:?}!"
                                        );
                                        let robot = ctx.create_robot(id);
                                        ctx.robots.push(robot);
                                        ctx.robots.last_mut().unwrap()
                                    }
                                };

                            step_agent(&mut state, robot, args.clone(), ctx.behavior_fn);

                            output_tx.send((id, state)).unwrap();
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
            soups: HashMap::new(),
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
    ) -> Vec<(RobotId, RobotState)> {
        let len = inputs.len();

        for (id, state) in inputs {
            let thread_index = self.robot_map.get(&id).unwrap();
            self.input_channels[*thread_index]
                .send(Some((id, state, args.clone())))
                .unwrap();
        }

        let mut results = BTreeMap::new();
        for _ in 0..len {
            let (id, state) = self.output_channel.recv().unwrap();
            results.insert(id, state);
        }
        results.into_iter().collect()
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

fn step_agent(
    state: &mut RobotState,
    robot: &mut Box<dyn botbrain::Robot>,
    args: Arc<StepArgs>,
    behavior_fn: BehaviorFn,
) {
    let StepArgs {
        agents,
        world,
        time,
        dt,
        msg_send_tx,
        pending_msgs,
    } = &*args;

    let dt = *dt;

    // Call the behavior function
    let (control, msgs) = behavior_fn(robot, *time);

    // Update the robot state
    {
        state.control = control;
        state.vel = state.control.speed * SPEED_MULTIPLIER;
        state.avel = state.control.steer * STEER_MULTIPLIER;

        // Update position of the robot
        let vel = Vec2::angled(state.pose.angle) * state.vel;
        state.pose.pos += vel * dt;
        state.pose.angle += state.avel * dt;

        // Set the new state of the robot
        robot.input_pose(RobotPose {
            pos: state.pose.pos,
            angle: state.pose.angle,
        });

        // Swap soups
        std::mem::swap(robot.get_debug_soup_mut(), &mut state.soup);
    }

    // Update postboxes
    {
        // Send messages
        for msg in msgs {
            let _ = msg_send_tx.send(msg).map_err(|e| {
                eprintln!("Error sending message: {:?}", e.to_string());
            });
        }

        // Receive messages
        robot.input_msgs(pending_msgs.clone());
    }

    // Update lidar data
    {
        let points = (0..LIDAR_RAYS)
            .map(|n| {
                let angle = n as f32 / LIDAR_RAYS as f32 * 2.0 * PI;
                let (distance, _) = cast_ray(
                    world,
                    agents,
                    state.pose.pos,
                    state.pose.angle + angle,
                    DIAMETER / 2.0,
                    LIDAR_RANGE,
                    &[Cell::SearchItem],
                );
                botbrain::LidarPoint { angle, distance }
            })
            .collect();

        robot.input_lidar(botbrain::LidarData::new(points));
    }

    // Update robot camera
    {
        let angle_step = CAM_FOV / (CAMERA_RAYS - 1) as f32;
        let points = (0..CAMERA_RAYS)
            .filter_map(|n| {
                let angle = n as f32 * angle_step - CAM_FOV / 2.0;
                let (distance, cell) = cast_ray(
                    world,
                    agents,
                    state.pose.pos,
                    state.pose.angle + angle,
                    DIAMETER / 2.0,
                    CAM_RANGE,
                    &[],
                );
                match cell {
                    Some(Cell::SearchItem) => {
                        let probability = (CAM_RANGE - distance) / CAM_RANGE;
                        Some((n, botbrain::CamPoint { angle, probability }))
                    }
                    _ => None,
                }
            })
            .collect::<Vec<_>>();

        // Consolidate points next to each other
        {
            let mut sparse_points = vec![];
            let mut adjacant = vec![];

            let consolidate_points = |adjacant: &mut Vec<_>, sparse_points: &mut Vec<_>| {
                if adjacant.is_empty() {
                    return;
                }

                let mut avg_point = adjacant.iter().fold(
                    CamPoint {
                        angle: 0.0,
                        probability: 0.0,
                    },
                    |a: CamPoint, (_, b): &(usize, CamPoint)| CamPoint {
                        angle: a.angle + b.angle,
                        probability: a.probability + b.probability,
                    },
                );
                avg_point.probability /= adjacant.len() as f32;
                avg_point.angle /= adjacant.len() as f32;
                sparse_points.push(avg_point);
                adjacant.clear();
            };

            for (n, point) in points {
                match adjacant.last() {
                    None => adjacant.push((n, point)),
                    Some((last_n, _)) if *last_n == n - 1 => adjacant.push((n, point)),
                    Some(_) => consolidate_points(&mut adjacant, &mut sparse_points),
                }
            }
            consolidate_points(&mut adjacant, &mut sparse_points);
            robot.input_cam(CamData::Points(sparse_points));
        }
    }

    // Resolve collisions
    collisions::resolve_robot_collisions(state, agents);
    collisions::resolve_border_collisions(state, world);
    collisions::resolve_world_collisions(state, world);
}
