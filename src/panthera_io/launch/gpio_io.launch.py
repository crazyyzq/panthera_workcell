from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    start_io = LaunchConfiguration('start_io')
    io_enabled = LaunchConfiguration('io_enabled')
    io_backend = LaunchConfiguration('io_backend')
    io_auto_run_workflow = LaunchConfiguration('io_auto_run_workflow')
    io_allow_output_writes = LaunchConfiguration('io_allow_output_writes')

    default_config_file = PathJoinSubstitution([
        FindPackageShare('panthera_io'),
        'config',
        'gpio_io.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='GPIO/DIDO parameter file',
        ),
        DeclareLaunchArgument('start_io', default_value='false'),
        DeclareLaunchArgument('io_enabled', default_value='false'),
        DeclareLaunchArgument('io_backend', default_value='sysfs'),
        DeclareLaunchArgument('io_auto_run_workflow', default_value='false'),
        DeclareLaunchArgument('io_allow_output_writes', default_value='false'),
        Node(
            package='panthera_io',
            executable='gpio_io_node',
            name='gpio_io_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'enabled': ParameterValue(io_enabled, value_type=bool),
                    'backend': io_backend,
                    'auto_run_workflow': ParameterValue(io_auto_run_workflow, value_type=bool),
                    'allow_output_writes': ParameterValue(
                        io_allow_output_writes,
                        value_type=bool,
                    ),
                },
            ],
            condition=IfCondition(start_io),
        ),
    ])
