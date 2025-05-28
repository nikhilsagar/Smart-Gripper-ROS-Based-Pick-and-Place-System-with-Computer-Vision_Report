# Smart Gripper: ROS-Based Pick-and-Place System with Computer Vision

This project implements a ROS2-based robotic arm capable of performing pick-and-place tasks using computer vision. The system uses Gazebo for simulation, MoveIt for motion planning, and a custom inverse kinematics (IK) action server for trajectory control.

---

## 📁 Workspace Preparation

Before starting, build and source your workspace:

```bash
source install/setup.bash   # Inside your workspace
colcon build                # Inside your workspace
```

---

## 🚀 Launch Instructions

### 🏗️ Start Gazebo

```bash
ros2 launch robot_arm_description gazebo.launch.py
ros2 launch robot_arm_controller controller.launch.py
```

### 🤖 Start MoveIt

```bash
ros2 launch moveit3 move_group.launch.py
ros2 launch moveit3 moveit_rviz.launch.py
```

### 🧠 Start Action Server

```bash
ros2 run moveit_scripts action_server
```

---

## 🧪 Testing the Action Server

Send a goal to the custom IK action server:

```bash
ros2 action send_goal /follow_ik_trajectory control_msgs/action/FollowJointTrajectory 
"{trajectory: { points: [{positions: [0.2, 0.25, 0.2, -0.75]}]}}"
```

- The goal is formatted as: `[x, y, z, orientation]`
- `orientation` = end effector’s angle relative to the horizontal plane (in radians)

---

## 📍 Current End Effector Pose

To view the live pose of the end effector:

```bash
ros2 run tf2_ros tf2_echo base_link dummy_link
```

---

## ⚠️ Important Configuration Change

Make sure to update the following line in the `robot_arm.urdf` file inside the `robot_arm_description` package:

```xml
<parameters>/home/ahmed/ros2_ws/src/robot_arm_controller/config/robot_arm_controllers.yaml</parameters>
```

> Replace with the correct path to your own `robot_arm_controllers.yaml` file.

---

## 🧠 How the Action Server Works

The custom action server:

- Calculates the position-only IK (joints 1–3) using MoveIt's `getPositionIK()`
- Determines the wrist joint angle (joint 4) needed to achieve the desired pose
- Sends the computed joint values to MoveIt's `setJointValueTarget()`

> **Why not use full IK?**  
MoveIt's position-and-orientation IK plugin was unreliable due to URDF inaccuracies. This workaround uses position-only IK with a compensation function that reduces final error to within **5 mm** and **2°**.

---

## 🔌 Serial Communication (for hardware interface)

```bash
minicom -b 115200 -D /dev/ttyACM0
```

Make sure UVC video drivers are loaded:

```bash
sudo modprobe uvcvideo
```

---


