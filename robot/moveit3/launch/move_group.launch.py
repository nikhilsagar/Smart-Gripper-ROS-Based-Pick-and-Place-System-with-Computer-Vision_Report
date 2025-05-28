from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="moveit3").to_moveit_configs()

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 1.0},  # Set to 1.0 (or 2.0 if needed)
            {"robot_description_planning.default_velocity_scaling_factor": 1.0},      # ✅ Add this
            {"robot_description_planning.default_acceleration_scaling_factor": 1.0},  # ✅ Add this
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
        ],
        # parameters=[
        #     moveit_config.to_dict(),
        #     {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
        #     {"publish_robot_description_semantic": True},
        #     {"use_sim_time": True},
        # ],
    )

    return LaunchDescription(
        [move_group_node]
    )
