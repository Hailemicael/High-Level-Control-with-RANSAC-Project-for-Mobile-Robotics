# High-Level Control with RANSAC for Mobile Robotics

This project implements a high-level control system for the Turtlebot3 robot, enabling autonomous wall-following behavior using ROS 2 and the RANSAC (Random Sample Consensus) algorithm for robust data processing.

## Authors

- **Hailemicael Lulseged Yimer**
- **Fitsum Sereke Tedlla**

*University of Verona, 2024*

## Project Overview

Our project focuses on developing a sophisticated control system for mobile robots using ROS 2 and RANSAC. The aim is to enhance the Turtlebot3's ability to follow walls autonomously by improving wall detection accuracy and robustness. The project demonstrates how high-level control can be used to manage complex behaviors through simplified sub-tasks and advanced data processing techniques.

## High-Level Control Tools

High-level control systems abstract complex sensor data processing to enable robots to perform advanced tasks by combining basic actions. Key tools and concepts used in high-level control include:

- **Finite State Machines (FSMs):** Manage the robot’s behavior by transitioning between different states based on sensor inputs.
- **RANSAC Algorithm:** Enhances data robustness and accuracy by fitting models to data while disregarding outliers.

## FSM for Wall Following

### States and Transitions

The Finite State Machine (FSM) used in this project manages the robot's wall-following behavior through the following states:

1. **Find a Wall:** The robot uses LiDAR data to search for a wall.
2. **Follow the Wall:** The robot maintains a specific distance from the wall while moving forward.
3. **Align Left:** The robot rotates to align the wall correctly, typically to the right if turning left.

#### Transition Rules

- **E1:** Transition to "Find Wall" when the right LiDAR sensor detects a wall beyond a defined threshold.
- **E2:** Transition to "Align Left" if the front LiDAR sensor detects a wall or if both the front and right LiDAR regions are too close.
- **E3:** Transition to "Follow Wall" if both the front and left LiDAR sensors detect walls.

### Implementation of Primitives

- **Find Wall:** Move the robot forward with specific linear and angular velocities to locate the wall.
- **Align Left:** Rotate the robot in place with a specified angular velocity to align it with the wall.
- **Follow Wall:** Continue moving forward while keeping a set distance from the wall.

## RANSAC Integration

RANSAC is employed to enhance the robustness of wall detection and distance measurements in this project. 

### How RANSAC is Applied

1. **Wall Detection:** Use RANSAC to fit a line to the LiDAR data, filtering out noise and accurately determining the wall's position.
2. **Distance Measurement:** Calculate the distance between the robot and the wall based on the line fitted by RANSAC.
3. **Update FSM:** Refine FSM transitions using the improved wall detection and distance measurements provided by RANSAC.

## Visuals

### FSM Diagram

![FSM Diagram](https://github.com/Hailemicael/High-Level-Control-with-RANSAC-Project-for-Mobile-Robotics/blob/main/Image/HighLabelCOntroller.PNG)

*Description: Diagram showing the states and transitions of the FSM used for wall-following.*

### Wall Following Scenario

![Wall Following](https://github.com/Hailemicael/High-Level-Control-with-RANSAC-Project-for-Mobile-Robotics/blob/main/Image/HighLabelWallFollow1.PNG)

*Description: Illustration of the robot following the wall, with LiDAR sensor readings and wall detection.*

### RANSAC Wall Detection

![RANSAC Wall Detection](https://github.com/Hailemicael/High-Level-Control-with-RANSAC-Project-for-Mobile-Robotics/blob/main/Image/HighLabelWallFollow.PNG)

*Description: Visualization of RANSAC applied to LiDAR data for improved wall detection.*


## Video Demonstrations

### HighLabelControlWithRansac

[![Watch HighLabelControlWithRansac on Vimeo](https://github.com/Hailemicael/High-Level-Control-with-RANSAC-Project-for-Mobile-Robotics/blob/main/Image/Screenshot%20from%202024-07-23%2018-07-23.png)](https://vimeo.com/990198283)

*Description: Demonstration of the high-level control system with RANSAC.*

You can also view the video directly on [Vimeo](https://vimeo.com/990198283).

### HighLabelllControlWithRansac
[![Watch HighLabelControlWithRansac on Vimeo](https://github.com/Hailemicael/High-Level-Control-with-RANSAC-Project-for-Mobile-Robotics/blob/main/Image/Screenshot%20from%202024-07-24%2001-17-26.png)](https://vimeo.com/990198008)

*Description: Demonstration of the high-level control system with RANSAC.*

You can also view the video directly on [Vimeo](https://vimeo.com//990198008).
## Key Features

- **Wall Detection:** Utilizes RANSAC for accurate wall detection.
- **State Machine Control:** Manages wall-following behavior with a Finite State Machine.
- **Real-Time Visualization:** Provides real-time plots of laser data and fitted lines.
- **ROS2 Integration:** Seamlessly integrates with ROS2 for robot control and sensor data processing.

## Installation

### Prerequisites

- **Operating System:** Ubuntu 20.04 or later (22.04 recommended)
- **ROS2:** Humble or later
- **Python:** 3.8 or later
- **Unity (Optional):** For simulation, if applicable

### ROS2 Installation

1. Follow the [official ROS2 installation guide](https://docs.ros.org/en/humble/Installation.html).
2. Install the desktop version for full functionality.

### Unity Installation

*Note: Unity is only necessary for simulation.*

1. Download and install Unity Hub from the [Unity Download Page](https://unity.com/download).
2. Install Unity version 2021.3 LTS (recommended).
3. Follow the [Unity-ROS2 integration guide](https://github.com/Unity-Technologies/ROS-TCP-Connector).

### Project Setup

1. **Create a ROS2 workspace:**
   ```sh
   mkdir -p ~/colcon_ws/src
   cd ~/colcon_ws/src

   ```

2. Clone the project repository:
   ```sh
   git clone https://github.com/Hailemicael/High-Level-Control-with-RANSAC-Project-for-Mobile-Robotics.git
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

## Code Structure

The codebase is organized into the following main components:

1. **`Turtlebot3HighLevelControl` Class:**
   - **Initializes ROS 2 Publishers, Subscribers, and Timers:** Sets up communication with ROS 2 topics and services, and manages timing for periodic tasks.
   - **Processes Laser Data and Applies RANSAC:** Handles incoming laser data, applies the RANSAC algorithm for line fitting, and extracts relevant information.
   - **Implements Control Logic Based on FSM States:** Executes the control logic according to the current state of the Finite State Machine (FSM).
   - **Handles Real-Time Plotting:** Provides real-time visualization of data and system states.

2. **Main Function:**
   - **Initializes the ROS 2 Node:** Sets up the main ROS 2 node for the application.
   - **Runs the Control Loop:** Executes the main loop that drives the control logic and state updates.
   - **Handles Graceful Shutdown:** Ensures proper shutdown procedures are followed to clean up resources and terminate processes cleanly.


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


For any questions or issues, don't hesitate to get in touch with the authors or open an issue in the project repository.
- hailemicaellulseged.yimer@studenti.univr.it
- fitsumsereke.tedla@studenti.univr.it
