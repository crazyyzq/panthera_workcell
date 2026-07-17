from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_hardware = LaunchConfiguration('start_hardware')
    control_mode = LaunchConfiguration('control_mode')
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
            default_value='position_velocity'),
        DeclareLaunchArgument('catalog_file', default_value=default_catalog),
        DeclareLaunchArgument('default_speed_scale', default_value='0.20'),
        hardware_launch,
        static_tf_launch,
        motion_launch,
    ])
