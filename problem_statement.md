# Coordinated Robot Search
University of Southern Denmark - Bachelor Project 2025

## Introduction
Multi-robot-systems (MRS) have a wide range of use cases, and can be used to create fault tolerant and reactive systems that can adapt to highly dynamic environments. Applications include storage management, formation control, environment mapping and many more. Generally, a cooperative MRS tries to coordinate robot actions to reach a common goal. Many such systems work by delegating tasks to robots which further the progress of the system. Researchers have developed many approaches to assigning tasks to robots optimally, as shown in State of the Art section. Commonly, robots also need to share observations like positions and sensor data with other robots in the system. Communication is not always reliable and can therefore impose restrictions on the distance and objects between robots in the system.

## State of the Art
A paper review by Janardan Kumar Verma & Virender Ranga outlines the current state MRS and provides a taxonomy for describing types of MRS and the problems and solutions related to them. This article mentions several approaches solving the task assignment problem. Notable approaches include market-based, dynamic programming (1), optimisation based, behavior based and deep reinforcement learning. Market based task assignment involves an auction where robots bid on a task and the highest bidding robot gets assigned the task which results in nearly optimal task assignment (2). Dynamic programming approaches seek to calculate the optimal task assignment given the location and capabilities of all robots in the system. This usually information about the entire MRS to be solved and can therefore only be used in centralized approaches. Behavior based approaches are usually inspired by nature and include ideas like laying down virtual “pheromones” or other signals which implicitly communicate through tasks to other robots. One paper found that “a coordinated group effort is possible without use of direct communication or robot differentiation” (3). This creates a system where less explicit communication is required. A whole other approach is deep reinforcement learning. Here, the MRS learns to reduce cost function and find the optimal policy for selecting tasks (4). Advantages include being adaptive to any environment at the cost of extensive training.

## Problem statement
The goal of this project is to create a decentralized MRS with the goal of locating a specified search item in a constrained search area. During the mission the environment is progressively explored and mapped. The robots should have their own state (a “world view”) which contains the environment as observed by the robots during the mission. This “world view” should be updated with a combination of self-observed data and received observations from other robots. As each robot maintains their own “world view”, it is vital that they are kept in sync as the mission progresses. This process should be optimized so the robots send a minimal amount of data to each other while still being able to track the world state accurately. Further communication limitations, such as restricted range or imperfect data transmission, may also be explored. The robots should calculate their movements based on their “world view” and position. The problem resides in making the robots behave in a way which facilitates an effective search. Each robot may be equipped with a camera for detecting the search item and a LiDAR to determine distances to objects for obstacle avoidance. All robots should run the same program and communicate their position and observations. The system should be simulated in a virtual environment using ROS2 and may be implemented on physical Turtlebot Burger robots depending on the complexity of implementation. Positioning of the robots is not the focus of this project, and could therefore be obtained directly from internal simulation transform frames, some pre-built localization algorithm like the ACML, or from a global positioning system in a possible physical implementation. It may be investigated how the system handles corrupted and lost communication packets and how it could be made resilient to these types of errors.

The concrete problems we want to solve are:

- How could a distributed algorithm for determining the behaviour of each robot be implemented such that the robots will conduct an effective search as a group?
- How can information be shared efficiently among robots in a way which minimizes the data transferred?
- How can ROS2 be utilized to simulate the system and validate the behavior algorithm and information sharing? Milestones

1. Simulation setup in ROS2
2. First working prototype
    2.1. Proposal for behavior algorithm
    2.2. Proposal for storing state/”world view” for each robot
    2.3. Proposal for data transfer protocol
    2.4. Determine initial software structure for the project
    2.5. Implementation of initial behavior algorithm
    2.6. Simulation of initial behavior algorithm
3. Benchmarking
    3.1. Determine parameters to be used for performance evaluation
    3.2. Automatically benchmark a behavior algorithm
4. Refine the behavior algorithm
    4.1. Propose a new behavior algorithm
    4.2. Implement new behavior algorithm
    4.3. Benchmark new algorithm
    4.4. Repeat until satisfied or out of time
5. Report
    5.1. Create outline
    5.1.1. What should be included and in what order
    5.2. Create figures
    5.2.1. Create figures manually
    5.2.2. Generate figures from simulation
    5.3. Write the bulk of report
    5.4. Refine
    5.4.1. Proofread
    5.5. Submit
