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
            default_value='75.0,105.0,135.0,135.0,75.0,75.0',
            description='MIT Kp for joint1..joint6',
        ),
        DeclareLaunchArgument(
            'mit_kd',
            default_value='5.5,5.5,5.5,5.5,5.5,5.5',
            description='MIT Kd for joint1..joint6',
        ),
        DeclareLaunchArgument(
            'mit_gravity_scale',
            default_value='0.0,1.04,1.10,1.52,0.0,0.0',
            description='Gravity feed-forward scale for joint1..joint6',
        ),
        DeclareLaunchArgument(
            'payload_mass_kg',
            default_value='0.0',
            description='Added tool mass in kg; zero preserves the URDF model',
        ),
        DeclareLaunchArgument(
            'payload_com_xyz_m',
            default_value='0.0,0.0,0.0',
            description='Added tool COM x,y,z in payload_frame, metres',
        ),
        DeclareLaunchArgument(
            'payload_frame',
            default_value='gripper_center',
            description='URDF frame used for the added tool COM',
        ),
        SetEnvironmentVariable(
            name='PANTHERA_MIT_KP',
            value=LaunchConfiguration('mit_kp'),
        ),
        SetEnvironmentVariable(
            name='PANTHERA_MIT_KD',
            value=LaunchConfiguration('mit_kd'),
        ),
        SetEnvironmentVariable(
            name='PANTHERA_MIT_GRAVITY_SCALE',
            value=LaunchConfiguration('mit_gravity_scale'),
        ),
        SetEnvironmentVariable(
            name='PANTHERA_PAYLOAD_MASS_KG',
            value=LaunchConfiguration('payload_mass_kg'),
        ),
        SetEnvironmentVariable(
            name='PANTHERA_PAYLOAD_COM_XYZ_M',
            value=LaunchConfiguration('payload_com_xyz_m'),
        ),
        SetEnvironmentVariable(
            name='PANTHERA_PAYLOAD_FRAME',
            value=LaunchConfiguration('payload_frame'),
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
