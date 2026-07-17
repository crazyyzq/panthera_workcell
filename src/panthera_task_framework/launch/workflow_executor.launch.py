from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    workflow_file = LaunchConfiguration('workflow_file')
    default_workflow = LaunchConfiguration('default_workflow')
    execute_motion = LaunchConfiguration('execute_motion')

    moveit_config = (
        MoveItConfigsBuilder(
            'panthera_ht_ros_description',
            package_name='panthera_ht_config',
        )
        .to_moveit_configs()
    )

    default_workflow_file = PathJoinSubstitution([
        FindPackageShare('panthera_task_framework'),
        'config',
        'cup_pick_place_workflows.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'workflow_file',
            default_value=default_workflow_file,
            description='YAML file that defines pose sources and workflows',
        ),
        DeclareLaunchArgument(
            'default_workflow',
            default_value='cup_pick_place',
            description='Workflow used by /run_default_workflow',
        ),
        DeclareLaunchArgument(
            'execute_motion',
            default_value='true',
            description='If false, the node plans only and skips arm/gripper execution',
        ),
        Node(
            package='panthera_task_framework',
            executable='workflow_executor_node',
            name='panthera_workflow_executor',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {
                    'workflow_file': workflow_file,
                    'default_workflow': default_workflow,
                    'execute_motion': ParameterValue(execute_motion, value_type=bool),
                    'arm_group': 'arm',
                    'hand_frame': 'gripper_center',
                    'base_frame': 'base_link',
                    'gripper_command_topic': '/gripper_controller/joint_trajectory',
                    'gripper_joint': 'L_finger_joint',
                    'gripper_motion_duration': 2.0,
                    'gripper_min_position': 0.0,
                    'gripper_max_position': 0.05,
                    'workflow_status_topic': '/workflow/status',
                    'velocity_scale': 0.10,
                    'acceleration_scale': 0.10,
                    'planning_time': 10.0,
                    'planning_attempts': 10,
                },
            ],
        ),
    ])
