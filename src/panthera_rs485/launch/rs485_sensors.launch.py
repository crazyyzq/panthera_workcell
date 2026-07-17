from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    start_laser = LaunchConfiguration('start_laser')
    laser_port = LaunchConfiguration('laser_port')
    laser_baudrate = LaunchConfiguration('laser_baudrate')
    laser_slave_id = LaunchConfiguration('laser_slave_id')
    laser_decode_mode = LaunchConfiguration('laser_decode_mode')
    laser_scale = LaunchConfiguration('laser_scale')
    laser_offset_mm = LaunchConfiguration('laser_offset_mm')
    laser_unit = LaunchConfiguration('laser_unit')
    laser_poll_rate_hz = LaunchConfiguration('laser_poll_rate_hz')
    laser_publish_pose = LaunchConfiguration('laser_publish_pose')

    start_state_signal = LaunchConfiguration('start_state_signal')
    state_port = LaunchConfiguration('state_port')
    state_baudrate = LaunchConfiguration('state_baudrate')
    state_slave_id = LaunchConfiguration('state_slave_id')
    state_register_address = LaunchConfiguration('state_register_address')
    state_auto_run_workflow = LaunchConfiguration('state_auto_run_workflow')

    default_config_file = PathJoinSubstitution([
        FindPackageShare('panthera_rs485'),
        'config',
        'rs485_devices.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='RS485 device parameter file',
        ),
        DeclareLaunchArgument('start_laser', default_value='true'),
        DeclareLaunchArgument('laser_port', default_value='/dev/ttyS4'),
        DeclareLaunchArgument('laser_baudrate', default_value='9600'),
        DeclareLaunchArgument('laser_slave_id', default_value='1'),
        DeclareLaunchArgument('laser_decode_mode', default_value='uint32_abcd'),
        DeclareLaunchArgument('laser_scale', default_value='0.1'),
        DeclareLaunchArgument('laser_offset_mm', default_value='0.0'),
        DeclareLaunchArgument('laser_unit', default_value='mm'),
        DeclareLaunchArgument('laser_poll_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('laser_publish_pose', default_value='false'),
        DeclareLaunchArgument('start_state_signal', default_value='false'),
        DeclareLaunchArgument('state_port', default_value='/dev/ttyS3'),
        DeclareLaunchArgument('state_baudrate', default_value='9600'),
        DeclareLaunchArgument('state_slave_id', default_value='1'),
        DeclareLaunchArgument('state_register_address', default_value='0'),
        DeclareLaunchArgument('state_auto_run_workflow', default_value='false'),
        Node(
            package='panthera_rs485',
            executable='laser_distance_node',
            name='laser_distance_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'port': laser_port,
                    'baudrate': ParameterValue(laser_baudrate, value_type=int),
                    'slave_id': ParameterValue(laser_slave_id, value_type=int),
                    'decode_mode': laser_decode_mode,
                    'scale': ParameterValue(laser_scale, value_type=float),
                    'offset_mm': ParameterValue(laser_offset_mm, value_type=float),
                    'unit': laser_unit,
                    'poll_rate_hz': ParameterValue(laser_poll_rate_hz, value_type=float),
                    'publish_pose': ParameterValue(laser_publish_pose, value_type=bool),
                },
            ],
            condition=IfCondition(start_laser),
        ),
        Node(
            package='panthera_rs485',
            executable='rs485_state_signal_node',
            name='rs485_state_signal_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'port': state_port,
                    'baudrate': ParameterValue(state_baudrate, value_type=int),
                    'slave_id': ParameterValue(state_slave_id, value_type=int),
                    'register_address': ParameterValue(state_register_address, value_type=int),
                    'auto_run_workflow': ParameterValue(state_auto_run_workflow, value_type=bool),
                },
            ],
            condition=IfCondition(start_state_signal),
        ),
    ])
