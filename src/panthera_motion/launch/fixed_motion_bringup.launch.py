from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_hardware = LaunchConfiguration('start_hardware')
    control_mode = LaunchConfiguration('control_mode')
    mit_kp = LaunchConfiguration('mit_kp')
    mit_kd = LaunchConfiguration('mit_kd')
    mit_gravity_scale = LaunchConfiguration('mit_gravity_scale')
    catalog_file = LaunchConfiguration('catalog_file')
    default_speed_scale = LaunchConfiguration('default_speed_scale')

    default_catalog = PathJoinSubstitution([
        FindPackageShare('panthera_motion'),
        'config',
        'motion_catalog.yaml',
    ])

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_ht_config'),
                'launch',
                'hardware.launch.py',
            ])
        ]),
        launch_arguments={'control_mode': control_mode}.items(),
        condition=IfCondition(start_hardware),
    )

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_ht_config'),
                'launch',
                'static_virtual_joint_tfs.launch.py',
            ])
        ]),
    )

    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_motion'),
                'launch',
                'motion_server.launch.py',
            ])
        ]),
        launch_arguments={
            'catalog_file': catalog_file,
            'default_speed_scale': default_speed_scale,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument(
            'control_mode',
            default_value='mit_gravity_compensation'),
        DeclareLaunchArgument(
            'mit_kp',
            default_value='60.0,60.0,60.0,60.0,60.0,60.0'),
        DeclareLaunchArgument(
            'mit_kd',
            default_value='5.5,5.5,5.5,5.5,5.5,5.5'),
        DeclareLaunchArgument(
            'mit_gravity_scale',
            default_value='0.0,1.0,1.5,0.0,0.0,0.0'),
        DeclareLaunchArgument('catalog_file', default_value=default_catalog),
        DeclareLaunchArgument('default_speed_scale', default_value='0.20'),
        SetEnvironmentVariable(name='PANTHERA_MIT_KP', value=mit_kp),
        SetEnvironmentVariable(name='PANTHERA_MIT_KD', value=mit_kd),
        SetEnvironmentVariable(
            name='PANTHERA_MIT_GRAVITY_SCALE',
            value=mit_gravity_scale),
        hardware_launch,
        static_tf_launch,
        motion_launch,
    ])
