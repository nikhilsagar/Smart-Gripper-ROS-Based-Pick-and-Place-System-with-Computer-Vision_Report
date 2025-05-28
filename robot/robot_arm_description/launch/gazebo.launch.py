import os
						
																   

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
															 
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
from pathlib import Path

def generate_launch_description():
    # Get the path to the 'robot_arm_description' package
    robot_arm_description_dir = get_package_share_path('robot_arm_description')

    # Define the path to the XACRO file
    urdf_path = os.path.join(robot_arm_description_dir, 'urdf', 'robot_arm.urdf.xacro')

    # Declare the launch argument for the model (XACRO file)
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=urdf_path,  # Default path to the XACRO file
        description="Path to the robot urdf file (XACRO)"
    )

	  # Set the Gazebo resource path (absolute path to resources)
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=str(Path(robot_arm_description_dir).parent.resolve())  # Resolving the absolute path
    )

    # Define the ROS distribution and physics engine settings
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"
    physics_engine = "--physics-engine gz-physics-bullet-featherstone-plugin" if ros_distro != "humble" else ""

    # Robot description parameter using the XACRO file and LaunchConfiguration
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration('model'),
            " is_ignition:=",
            is_ignition
            ]),  # Use the launch argument 'model'
        value_type=str
    )

  
    
    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, "use_sim_time": True}]
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
									   
        PythonLaunchDescriptionSource([os.path.join(get_package_share_path("ros_gz_sim"), "launch", "gz_sim.launch.py")]),
		   
        launch_arguments=[
            ("gz_args", [f"-v 4 {physics_engine}", "-r empty.sdf"])  # Pass the physics engine argument and an empty world
        ]
    )

    # Spawn the entity (robot) in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",  # The package from which to spawn the entity
        executable="create",   # The executable to launch (the command for creating an entity in Gazebo)
        output="screen",       # Output stream to the screen (for debugging or logging)
        arguments=[            # Arguments passed to the executable
            "-topic", "robot_description",  # The topic where the robot description is published
            "-name", "robotic_arm"  # Name for the robot entity in Gazebo (changed to "robotic_arm")
        ]
    )

    # Bridge the Clock topic between ROS and Gazebo
    gz_ros2_bridge = Node(
        package="ros_ign_bridge",          # The package responsible for bridging ROS and Ignition Gazebo
        executable="parameter_bridge",     # The executable that bridges topics between ROS and Ignition
        # name="gz_bridge"
        arguments=[                        # Arguments passed to the executable
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"  # This argument bridges the Clock topic
															 
																		
        ]
    )


    # Return the LaunchDescription with all nodes
    return LaunchDescription([
        model_arg,  # Launch argument for the robot model
        gazebo_resource_path,  # Set the environment variable for Gazebo resources
        robot_state_publisher_node,
        gazebo,  # Include the Gazebo launch description
        gz_spawn_entity,  # Spawn the robot entity in Gazebo
        gz_ros2_bridge  # Bridge the Clock topic
    ])
