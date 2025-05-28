import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    # Define the path to the XACRO file
    urdf_path = os.path.join(get_package_share_path('robot_arm_description'),
                             'urdf', 'robot_arm.urdf.xacro')

    # Declare the launch argument for the model (XACRO file)
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=urdf_path,  # Default path to the XACRO file
        description="Path to the robot urdf file (XACRO)"
    )

    # Robot description parameter using the XACRO file and LaunchConfiguration
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),  # Use the launch argument 'model'
        value_type=str
    )

    # Define the RViz configuration file path
    rviz_config_path = os.path.join(get_package_share_path('robot_arm_description'),
                                    'rviz', 'robot_arm_rviz.rviz')

    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description},
            {'use_sim_time': True}]
    )

    # Define the joint_state_publisher_gui node
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # Define the rviz2 node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Return the LaunchDescription with all nodes
    return LaunchDescription([
        model_arg,  # Launch argument for the robot model
        robot_state_publisher_node,
        joint_state_publisher_gui,
        rviz2
    ])
