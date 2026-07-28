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
    hardware_config_file = LaunchConfiguration('hardware_config_file')
    control_mode = LaunchConfiguration('control_mode')
    mit_kp = LaunchConfiguration('mit_kp')
    mit_kd = LaunchConfiguration('mit_kd')
    mit_gravity_scale = LaunchConfiguration('mit_gravity_scale')
    payload_mass_kg = LaunchConfiguration('payload_mass_kg')
    payload_com_xyz_m = LaunchConfiguration('payload_com_xyz_m')
    payload_frame = LaunchConfiguration('payload_frame')
    catalog_file = LaunchConfiguration('catalog_file')
    default_speed_scale = LaunchConfiguration('default_speed_scale')

    default_catalog = PathJoinSubstitution([
        FindPackageShare('panthera_motion'),
        'config',
        'motion_catalog.yaml',
    ])
    default_hardware_config = PathJoinSubstitution([
        FindPackageShare('panthera_ht_config'),
        'robot_param',
        'Follower_absolute.yaml',
    ])

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_ht_config'),
                'launch',
                'hardware.launch.py',
            ])
        ]),
        launch_arguments={
            'config_file': hardware_config_file,
            'control_mode': control_mode,
        }.items(),
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
            'hardware_config_file',
            default_value=default_hardware_config),
        DeclareLaunchArgument(
            'control_mode',
            default_value='mit_gravity_compensation'),
        DeclareLaunchArgument(
            'mit_kp',
            default_value='75.0,105.0,135.0,135.0,75.0,75.0'),
        DeclareLaunchArgument(
            'mit_kd',
            default_value='5.5,5.5,5.5,5.5,5.5,5.5'),
        DeclareLaunchArgument(
            'mit_gravity_scale',
            default_value='0.0,1.04,1.10,1.52,0.0,0.0'),
        DeclareLaunchArgument(
            'payload_mass_kg',
            default_value='0.0',
            description='Added tool mass in kg; zero preserves the URDF model'),
        DeclareLaunchArgument(
            'payload_com_xyz_m',
            default_value='0.0,0.0,0.0',
            description='Added tool COM x,y,z in payload_frame, metres'),
        DeclareLaunchArgument(
            'payload_frame',
            default_value='gripper_center',
            description='URDF frame used for the added tool COM'),
        DeclareLaunchArgument('catalog_file', default_value=default_catalog),
        DeclareLaunchArgument('default_speed_scale', default_value='0.20'),
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='1'),
        SetEnvironmentVariable(
            name='FASTDDS_BUILTIN_TRANSPORTS',
            value='UDPv4'),
        SetEnvironmentVariable(name='PANTHERA_MIT_KP', value=mit_kp),
        SetEnvironmentVariable(name='PANTHERA_MIT_KD', value=mit_kd),
        SetEnvironmentVariable(
            name='PANTHERA_MIT_GRAVITY_SCALE',
            value=mit_gravity_scale),
        SetEnvironmentVariable(
            name='PANTHERA_PAYLOAD_MASS_KG',
            value=payload_mass_kg),
        SetEnvironmentVariable(
            name='PANTHERA_PAYLOAD_COM_XYZ_M',
            value=payload_com_xyz_m),
        SetEnvironmentVariable(name='PANTHERA_PAYLOAD_FRAME', value=payload_frame),
        hardware_launch,
        static_tf_launch,
        motion_launch,
    ])
