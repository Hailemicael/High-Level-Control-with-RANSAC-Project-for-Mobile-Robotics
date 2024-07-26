import rclpy  # Import the rclpy library to work with ROS 2.
from rclpy.node import Node  # Import the Node class from rclpy to create a ROS 2 node.
from geometry_msgs.msg import Twist  # Import the Twist message from geometry_msgs to publish velocity commands.
from sensor_msgs.msg import LaserScan  # Import the LaserScan message from sensor_msgs to subscribe to laser scan data.
import numpy as np  # Import the numpy library for numerical operations.
import matplotlib  # Import matplotlib for plotting.
matplotlib.use('TkAgg')  # Set the backend for matplotlib to 'TkAgg' for interactive plots.
import matplotlib.pyplot as plt  # Import pyplot from matplotlib for plotting.
from matplotlib.animation import FuncAnimation  # Import FuncAnimation from matplotlib for animated plots.
from sklearn.linear_model import RANSACRegressor  # Import RANSACRegressor from sklearn for robust line fitting.
import time  # Import time for handling time-related tasks.

plt.ion()  # Turn on interactive mode for matplotlib to allow dynamic updates of plots.


class Turtlebot3HighLevelControl(Node):

    """
    Turtlebot3 High-Level Control System
    ------------------------------------
    This ROS 2 node is designed for controlling a Turtlebot3 robot to perform wall-following behavior. 
    Developed by Hailemicael Lulseged Yimer and Fitsum Sereke Tedla, this project integrates several advanced libraries and techniques to achieve autonomous navigation.
    
    Key Features:
    1. **ROS 2 Integration**: Utilizes ROS 2 for node creation, data publishing, and subscribing to sensor data.
    2. **Sensor Data Handling**: Receives and processes LaserScan data to detect and analyze obstacles around the robot.
    3. **Finite State Machine (FSM)**: Implements an FSM to handle different states: finding the wall, aligning with the wall, and following the wall.
    4. **RANSAC Algorithm**: Applies the RANSAC algorithm to fit lines to the laser scan data, which helps in detecting and tracking walls.
    5. **Real-time Visualization**: Uses Matplotlib for plotting and visualizing the laser scan data and the fitted lines, providing real-time feedback of the robot's environment.
    6. **Dynamic Control**: Adjusts the robot's movement based on the state of the FSM and the detected wall distance, ensuring consistent navigation and obstacle avoidance.

    Functionality:
    - **Initialization**: Sets up publishers, subscribers, and timers. Initializes plotting configurations.
    - **Laser Data Processing**: Converts laser scan data from polar to Cartesian coordinates, filters valid data, and performs RANSAC line fitting.
    - **Control Logic**: Based on the FSM state and sensor data, decides the robot's movement actions (e.g., move forward, turn left, follow the wall).
    - **Real-time Updates**: Continuously updates the plot with new sensor data and RANSAC line fittings.
    - **Error Handling**: Logs errors and warnings related to data processing and RANSAC fitting.
    - **State Transitions**: Changes the robot's state based on sensor readings and predefined conditions to adapt to different navigation scenarios.
    """

    def __init__(self):
        super().__init__('turtlebot3_HighLevelControl_node')  # Initialize the node with the name 'turtlebot3_HighLevelControl_node'.

        # Create a publisher to publish Twist messages to the '/cmd_vel' topic with a queue size of 1.
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        # Create a subscriber to the '/scan' topic to receive LaserScan messages and call the laser_callback method.
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)

        self.state_ = 0  # Initialize the state of the finite state machine (FSM) to 0 (Find wall).
        self.state_dict_ = {  # Dictionary mapping state numbers to state descriptions.
            0: 'Find wall',
            1: 'Align left',
            2: 'Follow the wall'
        }

        self.msg = Twist()  # Initialize the Twist message for velocity commands.
        self.th = 0.15  # Set the distance threshold to the wall.

        timer_period = 0.1  # Set the timer period to 0.1 seconds.
        # Create a timer to call the control_loop method at regular intervals defined by timer_period.
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.setup_plot()  # Initialize the plotting setup.

        # Add a timer to update the plot every 0.5 seconds.
        self.plot_timer = self.create_timer(0.5, self.update_plot)

        # Initialize data structures
        self.all_laser_points = []  # Initialize a list to store all laser points.
        self.plot_data = {  # Initialize a dictionary to store data for plotting.
            'x': [], 'y': [],
            'front_fit_x': [], 'front_fit_y': [],
            'left_fit_x': [], 'left_fit_y': [],
            'right_fit_x': [], 'right_fit_y': []
        }
        self.regions = {'front': 10, 'left': 10, 'right': 10}  # Initialize the distances to the walls in different regions.

    def setup_plot(self):
        self.fig, self.ax = plt.subplots()  # Create a new figure and axis for plotting.
        self.ax.set_xlim(-2, 2)  # Set the x-axis limits.
        self.ax.set_ylim(-2, 2)  # Set the y-axis limits.
        self.ax.set_aspect('equal')  # Set the aspect ratio to be equal.
        self.ax.grid(True)  # Enable grid on the plot.
        # Initialize scatter plots and lines for plotting.
        self.scatter = self.ax.scatter([], [], c='r', s=10, label='All Laser Points')
        self.inlier_scatter = self.ax.scatter([], [], c='g', s=20, label='RANSAC Inliers')
        self.line_front, = self.ax.plot([], [], 'b-', label='Front RANSAC')
        self.line_left, = self.ax.plot([], [], 'g-', label='Left RANSAC')
        self.line_right, = self.ax.plot([], [], 'm-', label='Right RANSAC')
        self.ax.set_title('Turtlebot3 Live View with RANSAC')  # Set the title of the plot.
        self.ax.set_xlabel('X (m)')  # Set the x-axis label.
        self.ax.set_ylabel('Y (m)')  # Set the y-axis label.
        self.ax.legend()  # Add a legend to the plot.


    def update_plot(self):
        try:
            if len(self.all_laser_points) > 0:  # If there are laser points available,
                x_data, y_data = zip(*self.all_laser_points)  # Unzip the laser points into x and y data.
                self.scatter.set_offsets(np.column_stack((x_data, y_data)))  # Update the scatter plot with new data.
            else:
                self.scatter.set_offsets(np.empty((0, 2)))  # Clear the scatter plot if no laser points are available.

            # Update the RANSAC lines for each region.
            for region in ['front', 'left', 'right']:
                if len(self.plot_data[f'{region}_fit_x']) > 0:
                    getattr(self, f'line_{region}').set_data(self.plot_data[f'{region}_fit_x'], self.plot_data[f'{region}_fit_y'])
                else:
                    getattr(self, f'line_{region}').set_data([], [])

            self.fig.canvas.draw()  # Redraw the figure.
            self.fig.canvas.flush_events()  # Process the GUI events.

            self.display_terminal_output()  # Display the terminal output.

        except Exception as e:
            self.get_logger().error(f'Error in update_plot: {str(e)}')  # Log any errors that occur.

    def control_loop(self):
        # Perform actions based on the current state of the FSM.
        if self.state_ == 0:
            self.find_wall()
        elif self.state_ == 1:
            self.align_left()
        elif self.state_ == 2:
            self.follow_the_wall()
        else:
            self.get_logger().error('Unknown state!')  # Log an error if the state is unknown.

        self.publisher_.publish(self.msg)  # Publish the velocity command.

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)  # Convert the laser scan ranges to a numpy array.
        angles = np.array([msg.angle_min + i * msg.angle_increment for i in range(len(ranges))])  # Calculate the angles for each range.

        valid_indices = (ranges > msg.range_min) & (ranges < msg.range_max)  # Filter out invalid ranges.
        valid_ranges = ranges[valid_indices]  # Get the valid ranges.
        valid_angles = angles[valid_indices]  # Get the valid angles.

        x = valid_ranges * np.cos(valid_angles)  # Convert polar coordinates to Cartesian coordinates (x).
        y = valid_ranges * np.sin(valid_angles)  # Convert polar coordinates to Cartesian coordinates (y).

        self.all_laser_points = list(zip(x, y))  # Store all valid laser points.

        data = np.column_stack((x, y))  # Prepare data for RANSAC.

        front_angle_half = 50 * np.pi / 180  # Convert 50 degrees to radians for the front region.
        side_angle = 85 * np.pi / 180  # Convert 85 degrees to radians for the side regions.

        angles_rad = np.arctan2(y, x)  # Calculate angles in radians.

        # Compute indices for each region.
        front_indices = np.abs(angles_rad) < front_angle_half
        left_indices = (angles_rad >= front_angle_half) & (angles_rad < front_angle_half + side_angle)
        right_indices = (angles_rad <= -front_angle_half) & (angles_rad > -front_angle_half - side_angle)

        # Fit RANSAC lines for each region.
        for region, indices in zip(['front', 'left', 'right'], [front_indices, left_indices, right_indices]):
            if np.sum(indices) > 2:  # Ensure there are at least 3 points for RANSAC fitting.
                region_x = x[indices]  # Extract x coordinates for the current region.
                region_y = y[indices]  # Extract y coordinates for the current region.
                try:
                    ransac = RANSACRegressor()  # Initialize the RANSAC regressor.
                    ransac.fit(region_x.reshape(-1, 1), region_y)  # Fit the RANSAC model to the data.
                    x_fit = np.linspace(min(region_x), max(region_x), 100)  # Generate x values for the fit line.
                    y_fit = ransac.predict(x_fit.reshape(-1, 1))  # Predict y values for the fit line.
                    self.plot_data[f'{region}_fit_x'] = x_fit  # Store x values for plotting.
                    self.plot_data[f'{region}_fit_y'] = y_fit  # Store y values for plotting.
                except Exception as e:
                    self.get_logger().warn(f'RANSAC fitting failed for {region}: {str(e)}')  # Log warning if RANSAC fails.
                    self.plot_data[f'{region}_fit_x'] = []  # Clear fit data on failure.
                    self.plot_data[f'{region}_fit_y'] = []  # Clear fit data on failure.

                # Update region distance.
                self.regions[region] = np.min(np.linalg.norm(np.column_stack((region_x, region_y)), axis=1)) if len(region_x) > 0 else 10  # Compute minimum distance to the wall for the current region or set default distance.

        self.take_action()  # Decide the action based on the updated region distances.

    def take_action(self):
        """
        Determines the appropriate action for the robot based on the current state and
        the distances to obstacles in the front, left, and right regions.
        """
        # If the robot is in state 1 or 2, and the distances to the right and front
        # are greater than the threshold, change state to 0 (Find wall).
        if (self.state_ == 1 or self.state_ == 2) and self.regions['right'] > self.th and self.regions['front'] > self.th:
            self.change_state(0)  #  Move to find the wall.
        
        # If the robot is in state 0 or 2, and the distance to the front is less than
        # the threshold but the distances to the left and right are greater than the threshold,
        # change state to 1 (Align left).
        elif (self.state_ == 0 or self.state_ == 2) and self.regions['front'] < self.th and self.regions['left'] > self.th and self.regions['right'] > self.th:
            self.change_state(1)  #  Align left with the wall.
        
        # If the robot is in state 0 or 2, and the distance to the front is less than
        # the threshold, the distance to the left is greater than the threshold, and the
        # distance to the right is less than the threshold, change state to 1 (Align left).
        elif (self.state_ == 0 or self.state_ == 2) and self.regions['front'] < self.th and self.regions['left'] > self.th and self.regions['right'] < self.th:
            self.change_state(1)  # Align left with the wall.
        
        # If the robot is in state 0 or 2, and the distance to the front is less than
        # the threshold, the distances to the left and right are also less than the threshold,
        # change state to 1 (Align left).
        elif (self.state_ == 0 or self.state_ == 2) and self.regions['front'] < self.th and self.regions['left'] < self.th and self.regions['right'] < self.th:
            self.change_state(1)  # Align left with the wall.
        
        # If the robot is in state 0 or 2, and the distance to the front is less than
        # the threshold, the distance to the left is less than the threshold, and the
        # distance to the right is greater than the threshold, change state to 1 (Align left).
        elif (self.state_ == 0 or self.state_ == 2) and self.regions['front'] < self.th and self.regions['left'] < self.th and self.regions['right'] > self.th:
            self.change_state(1)  #Align left with the wall.
        
        # If the robot is in state 0 or 1, and the distance to the front is greater than
        # the threshold, the distance to the left is greater than the threshold, and the
        # distance to the right is less than the threshold, change state to 2 (Follow wall).
        elif (self.state_ == 0 or self.state_ == 1) and self.regions['front'] > self.th and self.regions['left'] > self.th and self.regions['right'] < self.th:
            self.change_state(2)  # Follow the wall.

        # Log the current state and distances to the front, left, and right.
        self.get_logger().info(f"State: {self.state_} | Front: {self.regions['front']:.3f} | Left: {self.regions['left']:.3f} | Right: {self.regions['right']:.3f}")

    def change_state(self, state):
        """
        Changes the state of the robot and logs the transition.
        """
        if state != self.state_:
            # Log the transition to the new state.
            self.get_logger().info(f'Wall follower - [{state}] - {self.state_dict_[state]}')
            self.state_ = state  # Update the state.

    def find_wall(self):
        """
        Set the robot to move forward to find the wall.
        """
        self.msg.linear.x = 0.021  # Move forward with a small speed.
        self.msg.angular.z = 0.0  # No turning, just move straight.

    def align_left(self):
        """
        Set the robot to stop moving forward and turn left to align with the wall.
        """
        self.msg.linear.x = 0.0  # Stop moving forward.
        self.msg.angular.z = 0.21  # Turn left to align with the wall.

    def follow_the_wall(self):
        """
        Set the robot to move forward while following the wall.
        """
        self.msg.linear.x = 0.021  # Move forward with a small speed.
        self.msg.angular.z = 0.0  # Move straight, no turning.

    def stop_robot(self):
        """
        Stop the robot's movement by setting linear and angular velocities to zero.
        """
        self.msg.linear.x = 0.0  # Stop moving forward by setting the linear velocity to 0.
        self.msg.angular.z = 0.0  # Stop turning by setting the angular velocity to 0.
        self.publisher_.publish(self.msg)  # Publish the stop command to the '/cmd_vel' topic to execute the stop.

    def display_terminal_output(self):
        """
        Display diagnostic information about the robot's laser scan data and RANSAC line fitting.
        """
        # Log the first 10 laser points to the terminal.
        self.get_logger().info("All Laser Points:")
        for i, point in enumerate(self.all_laser_points[:10]):  # Display first 10 points from the laser scan data.
            self.get_logger().info(f"Point {i}: ({point[0]:.3f}, {point[1]:.3f})")
        
        # If there are more than 10 points, log how many more points there are beyond the first 10.
        if len(self.all_laser_points) > 10:
            self.get_logger().info(f"... and {len(self.all_laser_points) - 10} more points")

        # Log RANSAC line fitting results for each region (front, left, right).
        for region in ['front', 'left', 'right']:
            if len(self.plot_data[f'{region}_fit_x']) > 0:  # Check if there is RANSAC line data for the region.
                # Log the equation of the RANSAC line fitted to the data for the region.
                self.get_logger().info(f"{region.capitalize()} RANSAC Line: y = {self.plot_data[f'{region}_fit_y'][0]:.3f}x + {self.plot_data[f'{region}_fit_y'][0]:.3f}")
            else:
                # Log a message if no RANSAC line was fitted for the region.
                self.get_logger().info(f"No RANSAC line for {region}")

        # Log a separator line for clarity.
        self.get_logger().info("-------------------")

def main(args=None):
    """
    Main function to initialize and run the Turtlebot3 High-Level Control Node.
    """
    # Initialize the ROS client library
    rclpy.init(args=args)  # Initialize ROS client library for Python, passing command-line arguments if any.

    # Create an instance of the Turtlebot3HighLevelControl node
    turtlebot3_HighLevelControl_node = Turtlebot3HighLevelControl()

    try:
        # Enter a loop where the node will keep running and processing callbacks
        rclpy.spin(turtlebot3_HighLevelControl_node)  # This keeps the node active and responsive to callbacks until interrupted.
    except KeyboardInterrupt:
        # Handle a keyboard interrupt (Ctrl+C) gracefully
        turtlebot3_HighLevelControl_node.stop_robot()  # Stop the robot if interrupted by the user.
        print("Node terminated by user!")  # Inform the user that the node was terminated.
        time.sleep(0.5)  # Pause briefly to ensure all commands are processed before exiting.

    # Clean up after the node has been stopped
    turtlebot3_HighLevelControl_node.destroy_node()  # Cleanly destroy the node, releasing any resources.
    rclpy.shutdown()  # Shut down the ROS client library, terminating all communications.
    plt.close()  # Close the Matplotlib plot when the node shuts down to prevent hanging or errors.

if __name__ == '__main__':
    """
    Entry point for the script. Calls the main function to start the ROS node.
    """
    main()  # Execute the main function if this script is run directly.

# ==============================
# Code Summary and Explanation
# ==============================

# This script defines a ROS 2 node for high-level control of a Turtlebot3 robot. The robot is designed to perform wall-following behavior
# using laser scan data. Below is a summary of the key components and their functions:

# 1. **Imports:**
#    - `rclpy`: The ROS 2 client library for Python, used for creating nodes and managing ROS communication.
#    - `Node`, `Twist`, `LaserScan`: ROS 2 classes for creating nodes, publishing velocity commands, and subscribing to laser scan data.
#    - `numpy`: Library for numerical operations and data manipulation.
#    - `matplotlib`, `FuncAnimation`: Libraries for real-time plotting and animation of data.
#    - `RANSACRegressor`: For robust line fitting to detect walls in laser scan data.
#    - `time`: For handling time-related tasks.

# 2. **Class `Turtlebot3HighLevelControl`:**
#    - **Initialization (`__init__` method):** Sets up publishers, subscribers, timers, and initializes plotting.
#    - **Laser Data Processing (`laser_callback` method):** Converts laser scan data from polar to Cartesian coordinates, performs RANSAC line fitting, and updates region distances.
#    - **Control Logic (`control_loop` method):** Determines robot actions based on the current FSM state.
#    - **Action Handling (`take_action` method):** Decides and executes robot actions based on region distances and FSM state.
#    - **Plotting (`setup_plot` and `update_plot` methods):** Initializes and updates real-time plots of laser scan data and RANSAC lines.
#    - **State Management (`change_state` method):** Handles transitions between different robot states.

# 3. **Main Function (`main` method):**
#    - Initializes the ROS client library and creates an instance of the Turtlebot3HighLevelControl node.
#    - Runs the node in a loop to process callbacks.
#    - Handles keyboard interrupts to stop the robot and perform cleanup.

# 4. **Script Execution:**
#    - The `if __name__ == '__main__':` block ensures the `main` function is executed if the script is run directly.

