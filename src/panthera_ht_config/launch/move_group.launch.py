from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import os


def launch_setup(context, *args, **kwargs):
    # Get package path
    panthera_config_path = FindPackageShare('panthera_ht_config')

    # Build MoveIt configuration with HARDWARE-SPECIFIC files
    moveit_config = (
        MoveItConfigsBuilder("panthera_ht_ros_description", package_name="panthera_ht_config")
        .robot_description_semantic(file_path=os.path.join(
            panthera_config_path.perform(context),
            "config",
            "panthera_ht_ros_description_hardware.srdf"  # 硬件专用 SRDF
        ))
        .trajectory_execution(file_path=os.path.join(
            panthera_config_path.perform(context),
            "config",
            "moveit_controllers_hardware.yaml"  # 硬件专用 controllers
        ))
        .to_moveit_configs()
    )

    # Move group node with use_sim_time=false for real hardware
    move_group_configuration = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'publish_robot_description': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'monitor_dynamics': False,
        'capabilities': 'move_group/ExecuteTaskSolutionCapability',
        'trajectory_execution.allowed_start_tolerance': 10.0,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'use_sim_time': False,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
        ],
    )

    return [move_group_node]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
