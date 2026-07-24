from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = FindPackageShare('panthera_ht_config')
    default_config = PathJoinSubstitution([
        config_path,
        'robot_param',
        'Follower_absolute.yaml',
    ])
    moveit_launch = PathJoinSubstitution([
        config_path,
        'launch',
        'hardware_moveit_rviz.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to robot configuration YAML file',
        ),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'control_mode',
            default_value='mit_gravity_compensation',
            description='MIT mode with gravity feed-forward and position PD',
        ),
        DeclareLaunchArgument(
            'mit_kp',
            default_value='60.0,60.0,60.0,60.0,60.0,60.0',
            description='MIT Kp for joint1..joint6',
        ),
        DeclareLaunchArgument(
            'mit_kd',
            default_value='5.0,5.0,5.0,5.0,5.0,5.0',
            description='MIT Kd for joint1..joint6',
        ),
        SetEnvironmentVariable(
            name='PANTHERA_MIT_KP',
            value=LaunchConfiguration('mit_kp'),
        ),
        SetEnvironmentVariable(
            name='PANTHERA_MIT_KD',
            value=LaunchConfiguration('mit_kd'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file'),
                'control_mode': LaunchConfiguration('control_mode'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
