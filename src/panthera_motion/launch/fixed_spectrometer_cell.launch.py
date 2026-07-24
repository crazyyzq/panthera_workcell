from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    catalog_file = LaunchConfiguration('catalog_file')
    control_mode = LaunchConfiguration('control_mode')
    mit_kp = LaunchConfiguration('mit_kp')
    mit_kd = LaunchConfiguration('mit_kd')
    default_speed_scale = LaunchConfiguration('default_speed_scale')
    start_hardware = LaunchConfiguration('start_hardware')
    simulation = LaunchConfiguration('simulation')
    start_hmi = LaunchConfiguration('start_hmi')
    hmi_port = LaunchConfiguration('hmi_port')

    default_config = PathJoinSubstitution([
        FindPackageShare('panthera_spectrometer_cell'),
        'config',
        'spectrometer_cell.yaml',
    ])
    default_catalog = PathJoinSubstitution([
        FindPackageShare('panthera_motion'),
        'config',
        'motion_catalog.yaml',
    ])

    fixed_motion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_motion'),
                'launch',
                'fixed_motion_bringup.launch.py',
            ])
        ]),
        launch_arguments={
            'start_hardware': start_hardware,
            'control_mode': control_mode,
            'mit_kp': mit_kp,
            'mit_kd': mit_kd,
            'catalog_file': catalog_file,
            'default_speed_scale': default_speed_scale,
        }.items(),
    )

    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_spectrometer_cell'),
                'launch',
                'spectrometer_cell.launch.py',
            ])
        ]),
        launch_arguments={
            'config_file': config_file,
            'simulation': simulation,
        }.items(),
    )

    hmi = Node(
        package='panthera_web_hmi',
        executable='web_hmi_node',
        name='panthera_web_hmi',
        output='screen',
        parameters=[{
            'host': '0.0.0.0',
            'port': ParameterValue(hmi_port, value_type=int),
            'motion_catalog_path': catalog_file,
            'point_config_path': config_file,
        }],
        condition=IfCondition(start_hmi),
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('catalog_file', default_value=default_catalog),
        DeclareLaunchArgument(
            'control_mode',
            default_value='mit_gravity_compensation'),
        DeclareLaunchArgument(
            'mit_kp',
            default_value='60.0,60.0,60.0,60.0,60.0,60.0'),
        DeclareLaunchArgument(
            'mit_kd',
            default_value='5.0,5.0,5.0,5.0,5.0,5.0'),
        DeclareLaunchArgument('default_speed_scale', default_value='0.20'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('start_hmi', default_value='true'),
        DeclareLaunchArgument('hmi_port', default_value='8080'),
        fixed_motion,
        cell,
        hmi,
    ])
