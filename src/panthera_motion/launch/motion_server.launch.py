from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    catalog_file = LaunchConfiguration('catalog_file')
    controller_action = LaunchConfiguration('controller_action')
    default_speed_scale = LaunchConfiguration('default_speed_scale')

    moveit_config = (
        MoveItConfigsBuilder(
            'panthera_ht_ros_description',
            package_name='panthera_ht_config',
        )
        .to_moveit_configs()
    )

    default_catalog = PathJoinSubstitution([
        FindPackageShare('panthera_motion'),
        'config',
        'motion_catalog.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('catalog_file', default_value=default_catalog),
        DeclareLaunchArgument(
            'controller_action',
            default_value='/arm_controller/follow_joint_trajectory'),
        DeclareLaunchArgument('default_speed_scale', default_value='1.0'),
        Node(
            package='panthera_motion',
            executable='motion_server_node',
            name='panthera_motion_server',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {
                    'catalog_file': catalog_file,
                    'controller_action': controller_action,
                    'default_speed_scale': ParameterValue(
                        default_speed_scale, value_type=float),
                    'start_tolerance_rad': 0.05,
                    'joint_state_max_age_sec': 0.50,
                    'settled_velocity_rad_sec': 0.08,
                    'trajectory_start_delay_sec': 0.02,
                    'goal_position_tolerance_rad': 0.06,
                    'path_position_tolerance_rad': 0.15,
                    'pour_path_position_tolerance_rad': 0.20,
                    'wrist_path_position_tolerance_rad': 0.45,
                    'goal_velocity_tolerance_rad_sec': 0.05,
                },
            ],
        ),
    ])
