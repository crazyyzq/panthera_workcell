from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    result_file = LaunchConfiguration('result_file')

    default_config_file = PathJoinSubstitution([
        FindPackageShare('panthera_task_framework'),
        'config',
        'station_pose_commissioning.yaml',
    ])
    moveit_config = (
        MoveItConfigsBuilder(
            'panthera_ht_ros_description',
            package_name='panthera_ht_config',
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='Station pose commissioning YAML file',
        ),
        DeclareLaunchArgument(
            'result_file',
            default_value='',
            description='Optional JSONL output path for planning results',
        ),
        Node(
            package='panthera_task_framework',
            executable='pose_commissioning_node',
            name='panthera_pose_commissioning',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {
                    'config_file': config_file,
                    'result_file': result_file,
                    'stop_after_run': True,
                },
            ],
        ),
    ])
