from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    simulation = LaunchConfiguration('simulation')

    moveit_config = (
        MoveItConfigsBuilder(
            'panthera_ht_ros_description',
            package_name='panthera_ht_config',
        )
        .to_moveit_configs()
    )

    default_config_file = PathJoinSubstitution([
        FindPackageShare('panthera_spectrometer_cell'),
        'config',
        'spectrometer_cell.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='YAML configuration for the continuous spectrometer cell state machine',
        ),
        DeclareLaunchArgument(
            'simulation',
            default_value='false',
            description='Run with simulated robot, sensors and spectrometer adapters',
        ),
        Node(
            package='panthera_spectrometer_cell',
            executable='spectrometer_cell_node',
            name='panthera_spectrometer_cell',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {
                    'config_file': config_file,
                    'simulation_enabled': ParameterValue(simulation, value_type=bool),
                },
            ],
        ),
    ])
