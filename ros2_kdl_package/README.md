## 🤖 Project: Control your robot
**Course: Robotics Lab**

**Student: Leonardo Riccardi / P38000358**

---

## 🎯 Project Objective

The primary goal of this homework is to **develop kinematic and vision-based controllers** for a simulated robotic manipulator. This includes implementing KDL for **joint-limit avoidance** (using null-space projection) and utilizing the `aruco_ros` package to execute a **visual "Look-at-Point" task**.

---

## ⚙️ Prerequisites and Setup

To compile and run the project, ensure you have a standard **ROS 2** (e.g., Humble/Iron) workspace setup and the following dependencies installed.

### Dependencies Installation

It is necessary to have the essential dependencies, including those for plotting results:

```bash
sudo apt update
sudo apt install python3-pip
```
 Install Python packages for plotting
```
python3 -m pip install --user pandas matplotlib
pip install "numpy<2" pandas matplotlib
```
## Build
Clone this package in the `src` folder of your ROS 2 workspace.
```
https://github.com/leonardoricc2002/Homework_2.git

```
Build and source the setup files
```
colcon build 
source install/setup.bash
```
# 🚀 HOW TO LAUNCH
Launch the iiwa robot state publisher and Rviz2 for visualization.
```
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=false rviz:=true
```
On the terminal 2. Start the standard KDL controller (velocity_ctrl) and immediately execute the trajectory.

```
source install/setup.bash
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=velocity_ctrl auto_start:=true
```
Terminal 2. Generate and display plots (e.g., commanded velocities, joint positions) from the velocity_ctrl log file.
```
source install/setup.bash
python3 src/ros2_kdl_package/scripts/plot_results.py log_vel.csv
```
Terminal 2 .Start the controller with joint-limit avoidance enabled via null-space projection
```
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=velocity_ctrl_null auto_start:=true
```
Terminal 3.Generate plots specific to the null-space controller run.
```
python3 src/ros2_kdl_package/scripts/plot_results.py log_null.csv
```
# 🕹️ INTERACTION AND CONTROL----ACTION-CLIENT----
Launch the Action Server and wait for a goal (trajectory execution is initially suspended).Terminal 2 
```
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py auto_start:=false

```
Run the Action Client to send the trajectory goal and monitor feedback.Open another terminal and type:
```
ros2 run ros2_kdl_package linear_traj_client
```
## LAUNCH GAZEBO. Vision-based control
Open Terminal 1. This starts the robot in the Gazebo world containing the ArUco marker.
```
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true rviz:=false
```
Open Terminal 2 to verify that the camera is active and detecting the marker.
```
ros2 run rqt_image_view rqt_image_view 
```
Open Terminal 3 to activate the vision_ctrl. The robot will attempt to align its camera with the marker.
```
source install/setup.bash
ros2 launch ros2_kdl_package ros2_kdl_node.launch.py ctrl:=vision_ctrl auto_start:=true
```
Call the Gazebo service to move the ArUco marker; the robot should immediately react to maintain visual alignment.

```
source install/setup.bash
ros2 service call /world/iiwa_aruco_world/set_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: 'aruco_marker_static_instance'}, pose: {position: {x: 0.7, y: 0.1, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```
