# High-Level Control with RANSAC for Mobile Robotics

This project implements a high-level control system for a Turtlebot3 robot, enabling autonomous wall-following behavior using ROS 2 and advanced data processing techniques (Ransac).

## Table of Contents
- [Authors](#authors)
- [Project Overview](#project-overview)
- [Key Features](#key-features)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [ROS2 Installation](#ros2-installation)
  - [Unity Installation](#unity-installation)
  - [Project Setup](#project-setup)
- [Running the Project](#running-the-project)
- [Customization and Improvement](#customization-and-improvement)
- [Troubleshooting](#troubleshooting)
- [Additional Resources](#additional-resources)

## Authors

Hailemicael Lulseged Yimer and Fitsum Sereke Tedlla (University of Verona, 2024)

## Project Overview

Our project focuses on implementing high-level control in robotics using a RANSAC (Random Sample Consensus) algorithm to enhance wall-following behavior. High-level control involves managing complex tasks by breaking them down into simpler sub-tasks. In this project, we use RANSAC to improve the accuracy and robustness of wall detection and navigation.

## High-Level Control Tools

High-level control solutions abstract complex sensor data processing and enable robots to perform advanced tasks by combining basic actions. The tools commonly used for high-level control include:

- **Finite State Machines (FSM)**
- **Behavior Trees**
- **Petri Nets**

For this project, we utilize **Finite State Machines (FSM)** due to their simplicity and effectiveness in managing the robot's tasks.

## FSM for Wall Following

### States and Transitions

Our FSM for wall-following includes the following states and transitions:

1. **Find a Wall**: The robot searches for a wall using lidar data.
2. **Follow the Wall**: The robot maintains a specific distance from the wall while moving forward.
3. **Align Left**: The robot rotates to ensure the wall is positioned correctly (to the right if turning left).

#### Transition Rules

- **E1**: Transition to "Find Wall" if the right lidar region detects a wall (greater than the threshold).
- **E2**: Transition to "Align Left" if the front lidar region detects a wall or if both front and right lidar regions are too close.
- **E3**: Transition to "Follow Wall" if the front and left lidar regions detect walls.

### Implementation of Primitives

- **Find Wall**: Move the robot with defined linear and angular velocities.
- **Align Left**: Rotate the robot in place with a specified angular velocity.
- **Follow Wall**: Move forward while maintaining a defined distance from the wall.

## RANSAC Integration

In our project, RANSAC is used to enhance the wall-following behavior. The integration of RANSAC improves the robustness of wall detection and distance measurements.

### How RANSAC is Applied

1. **Wall Detection**: Use RANSAC to robustly fit a line to lidar data, filtering out noise and accurately identifying the wall's position.
2. **Distance Measurement**: Compute the distance from the robot to the wall based on the RANSAC-fitted line.
3. **Update FSM**: Refine FSM transitions using the improved wall detection and distance measurements provided by RANSAC.

## Visuals

### FSM Diagram

![FSM Diagram](path/to/fsm-diagram.png)

*Description: Diagram showing the states and transitions of the FSM used for wall-following.*

### Wall Following Scenario

![Wall Following](path/to/wall-following-scenario.png)

*Description: Illustration of the robot following the wall, with lidar sensor readings and wall detection.*

### RANSAC Wall Detection

![RANSAC Wall Detection](path/to/ransac-wall-detection.png)

*Description: Visualization of RANSAC applied to lidar data for improved wall detection.*


## Key Features

- Wall detection using RANSAC algorithm
- State machine-based control for wall following
- Real-time visualization of laser data and fitted lines
- ROS2 integration for robot control and sensor data processing

## Installation

### Prerequisites

- Ubuntu 20.04 or later (22.04 recommended)
- ROS2 Humble or later
- Python 3.8 or later
- Unity (for simulation, if applicable)

### ROS2 Installation

1. Follow the official ROS2 installation guide: [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)
2. Make sure to install the desktop version for full functionality

### Unity Installation

> Note: This step is only necessary if you plan to use simulation.

1. Download and install Unity Hub from: [Unity Download Page](https://unity.com/download)
2. Install the appropriate Unity version (2021.3 LTS recommended)
3. Follow the Unity-ROS2 integration guide: [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)

### Project Setup

1. Create a ROS2 workspace:
   ```sh
   mkdir -p ~/colcon_ws/src
   cd ~/colcon_ws/src
   ```

2. Clone the project repository:
   ```sh
   git clone https://Hailemicael/Turtlebot3-High-Level-Control-System-with-Ransac.git]
   ```

3. Install dependencies:
   ```sh
   cd ~/cd colcon_ws/
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the workspace:
   ```sh
   colcon build --symlink-install
   ```

5. Source the workspace:
   ```sh
   source ~/ros2_ws/install/setup.bash
   ```

## Running the Project

1. Launch the Robot:
   - For a real Turtlebot3:
     ```sh
     ros2 launch turtlebot3_bringup robot.launch.py

2. Launch the High-Level Control Node:
   ```sh
   cd colcon_ws/
   ~/colcon build
   ~/. install/setup.bash
   ros2 run turtlebot3_HighLevelControl turtlebot3_HighLevelControl 
   ```

## Customization and Improvement

Future students can improve this project by:

1. Implementing more sophisticated control algorithms (e.g., PID)
2. Incorporating SLAM for mapping and localization
3. Extending the visualization with 3D mapping or path planning displays

## Troubleshooting

- If you encounter "module not found" errors, ensure all dependencies are installed and the workspace is properly sourced
- For visualization issues, check if all necessary ROS2 topics are being published
- For robot control problems, verify that the correct ROS2 parameters are set for your specific robot model

## Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Turtlebot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [RANSAC Algorithm](https://en.wikipedia.org/wiki/Random_sample_consensus)

For any questions or issues, please contact the authors or open an issue in the project repository.
