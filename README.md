# 🤖 Project: ARMANDO Robotic Manipulator Simulation (4 DoF)
Course: Robotic Lab 
Student: Leonardo Riccardi / P38000358 

# 🎯 Project Objective
The objective of this project is the creation of a ROS 2 workspace containing the necessary packages for the simulation of a 4 Degrees of Freedom (DoF) robotic manipulator, named Armando, within the Gazebo simulation environment, using ROS 2 Humble and ros2_control.

# 🛠️ Prerequisites
To compile and run the project, it is necessary to have the essential dependencies for ROS 2 and Gazebo installed. After logging into your container or virtual machine, be sure to install:

```bash
# Package update
$ sudo apt update
# Optional GUI tools for debugging and visualization
$ sudo apt install ros-humble-joint-state-publisher-gui
$ sudo apt install ros-humble-rqt-image-view
```

# 3. 🏗️ BUILD
Cloning the Repository

Clone this repository into the desired folder (e.g., ~/ros2_ws):

```bash
$ git clone https://github.com/leonardoricc2002/rl25_homework1.git
$ colcon build
$ source install/setup.bash
```
# 🚀 HOW TO LAUNCH
4.1. Visualization Robot (Rviz2)

To see only the kinematic model of the robot and its frames in RViz2:

```bash 
$ ros2 launch armando_description armando_display.launch.py
```
Complete Simulation (Gazebo)
The main launchfile loads the robot in Gazebo and starts the controllers (ros2_control).

Default Control Mode (Position): Starts the joint position controller.
```bash
$ ros2 launch armando_gazebo armando_world.launch.py
```
Trajectory Mode (requires external node): Starts the joint trajectory controller.
```bash
$ ros2 launch armando_gazebo armando_world.launch.py controller_mode:=trajectory
```
# 5. 🕹️ INTERACTION AND CONTROL
5.1. Manual Position Control (Test)

To send a specific position setpoint to the joints (only in position mode), where the values are in the order [j0, j1, j2, j3] in radians:
Open a new terminal and run the source command.
Send the commands:
```bash
# Zero Position (to stabilize the robot)
$ ros2 topic pub --rate 10 /arm_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}"
```
```bash
# Movement Example (Base 90°, Shoulder 45°, Elbow -15°)
$ ros2 topic pub --once /arm_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.57, 0.78, -0.26, 0.0]}"
```
Launching the Trajectory Node (High-Level Control)
This node is required when launching Gazebo with controller_mode:=trajectory.

Function: The arm_controller_node implements the logic for [Inverse Kinematics / Trajectory Generation] and publishes the commands on the correct trajectory controller topic to move the robot autonomously.
```bash
$ ros2 run armando_controller arm_controller_node
```

To visualize the real-time video feed from the simulated camera in Gazebo: Launch rqt_image_view (after running source in a new terminal) and select the /camera/image topic:
```bash
$ ros2 run rqt_image_view rqt_image_view
```
# 🔍 VERIFICATION AND DEBUGGING
To confirm correct operation and monitor the system status:
Joint State Monitoring

To visualize real-time position, velocity, and effort data from the simulator:

```bash
$ ros2 topic echo /joint_states
```
Active Controller Verification

To confirm that the controllers have been loaded and started correctly:
```bash
$ ros2 control list_controllers
```
To see all running nodes:
```bash
$ ros2 node list
```
