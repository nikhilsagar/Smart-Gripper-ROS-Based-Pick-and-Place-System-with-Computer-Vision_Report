# Smart-Gripper-ROS-Based-Pick-and-Place-System-with-Computer-Vision_Report
PREPARE WORKSPACE:
source install/setup.bash #Inside your workspace
colcon build              #Inside your workspace

START GAZEBO:
ros2 launch robot_arm_description gazebo.launch.py
ros2 launch robot_arm_controller controller.launch.py

START MOVEIT:
ros2 launch moveit3 move_group.launch.py
ros2 launch moveit3 moveit_rviz.launch.py

START ACTION SERVER
ros2 run moveit_scripts action_server

TEST ACTION SERVER:
ros2 action send_goal /follow_ik_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: { points: [{positions: [0.2, 0.25, 0.2, -0.75]}]}}"
# Formatted as [x, y, z, orientation], the orientation is the end effectors angle relative to the horizontal plane in radians

CURRENT END EFFECTOR POSITION/ORIENTATION:
ros2 run tf2_ros tf2_echo base_link dummy_link

Make sure to change the following line in the robot_arm.urdf inside the robot_arm_description package
<parameters>/home/ahmed/ros2_ws/src/robot_arm_controller/config/robot_arm_controllers.yaml</parameters>
----------------------------------------------------------
The action server calculates where the wrist joints needs to be to achieve the desired end effector position and orientation, calculates the position only IK up to link_4 (which has the same origin of frame as joint_4, the wrist) with getPositionIK(), which returns the values for joints 1-3, calculates the exact wrist angle needed (joint_4) to achieve the desired dummy_link (end effector origin) position and orientation and finally send the 4 joint values to setJointValueTarget()

The action server basically calculates orientation and position inverse kinematics using moveit's position only mode since moveit's built in position and orientation IK plugins have failed; this approach isn't as accurate, likely due to an innacurate representation of the URDF model inside the program, but an added compensation function reduced the error to a maximum of 5mm and 2 degrees.


minicom -b 115200 -D /dev/ttyACM0



sudo modprobe uvcvideo
