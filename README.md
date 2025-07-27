# Battery-Constrained Frontier-Based Exploration using RRT

This project implements an energy-aware frontier exploration algorithm using the Rapidly-Exploring Random Tree (RRT) planner for autonomous robots. A battery constraint is introduced, allowing the robot to prioritize charging over exploration when battery levels drop below a critical threshold. The system is designed and tested using ROS and Gazebo with a Kobuki differential-drive robot in a simulated indoor house environment.

## Project Overview

- **Objective:** Efficiently explore a 2D environment while accounting for limited battery capacity.
- **Approach:** Extend frontier-based exploration by integrating a battery threshold mechanism into the RRT path planner.
- **Simulation:** Implemented in ROS + Gazebo using the Kobuki robot in a house world.

## Key Features

- RRT-based frontier exploration planner
- Battery-aware decision-making logic
- Autonomous navigation to charging stations
- Exploration resumption post-recharge
- Simulation tested with multiple scenarios:
  - Single charging station
  - Multiple charging stations
  - Varying exploration radii


## Algorithm Logic

When the robot is exploring:
- If **battery > threshold**: Continue frontier exploration using RRT.
- If **battery ≤ threshold**: Pause exploration, navigate to nearest charging station.
- After charging: Resume exploration from last known state.

## Simulation Setup

- **Robot:** Kobuki Differential Drive Robot
- **Frameworks:** ROS (Melodic/Noetic), Gazebo
- **Environment:** Indoor house map with charging stations
- **Map:** 2D occupancy grid (-1 for unknown, 0 for free, 1 for obstacles)

## Demonstration Videos

Watch the full simulation and results on YouTube: 
[Battery-Constrained Frontier Exploration – Playlist](https://www.youtube.com/watch?v=TS02ACSNanM&list=PLKMd3UD2lxcJ2C0dGKKYD6BFzLX10Sb0s)

## Credits

- **Simulation Environment:** Based on [hasauino/rrt_exploration](https://github.com/hasauino/rrt_exploration), modified and extended to support energy-aware planning.
- **Authors:** Hariharan Sureshkumar*, Nicholas Blanchard and Kodanda Rama Naidu Dumpala
- **Institution:** Northeastern University