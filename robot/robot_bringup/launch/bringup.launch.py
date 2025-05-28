from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths to the other launch files
    robot_description_dir = get_package_share_directory('robot_arm_description')
    robot_controller_dir = get_package_share_directory('robot_arm_controller')
    moveit_dir = get_package_share_directory('moveit3')

    description_launch = os.path.join(robot_description_dir, 'launch', 'gazebo.launch.py')
    controller_launch = os.path.join(robot_controller_dir, 'launch', 'controller.launch.py')
    move_group_launch = os.path.join(moveit_dir, 'launch', 'move_group.launch.py')
    rviz_launch = os.path.join(moveit_dir, 'launch', 'moveit_rviz.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(controller_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(move_group_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz_launch)),
    ])
