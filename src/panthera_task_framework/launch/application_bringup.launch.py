from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    workflow_file = LaunchConfiguration('workflow_file')
    default_workflow = LaunchConfiguration('default_workflow')
    execute_motion = LaunchConfiguration('execute_motion')

    start_hardware = LaunchConfiguration('start_hardware')
    start_workflow = LaunchConfiguration('start_workflow')
    start_laser = LaunchConfiguration('start_laser')
    start_state_signal = LaunchConfiguration('start_state_signal')
    start_io = LaunchConfiguration('start_io')
    rs485_config_file = LaunchConfiguration('rs485_config_file')

    rviz = LaunchConfiguration('rviz')
    control_mode = LaunchConfiguration('control_mode')
    laser_port = LaunchConfiguration('laser_port')
    state_port = LaunchConfiguration('state_port')
    state_auto_run_workflow = LaunchConfiguration('state_auto_run_workflow')
    io_enabled = LaunchConfiguration('io_enabled')
    io_backend = LaunchConfiguration('io_backend')
    io_auto_run_workflow = LaunchConfiguration('io_auto_run_workflow')
    io_allow_output_writes = LaunchConfiguration('io_allow_output_writes')

    default_workflow_file = PathJoinSubstitution([
        FindPackageShare('panthera_task_framework'),
        'config',
        'cup_pick_place_workflows.yaml',
    ])
    default_rs485_config_file = PathJoinSubstitution([
        FindPackageShare('panthera_rs485'),
        'config',
        'rs485_devices.yaml',
    ])

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_ht_config'),
                'launch',
                'hardware_moveit_rviz.launch.py',
            ])
        ]),
        launch_arguments={
            'rviz': rviz,
            'control_mode': control_mode,
        }.items(),
        condition=IfCondition(start_hardware),
    )

    workflow_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_task_framework'),
                'launch',
                'workflow_executor.launch.py',
            ])
        ]),
        launch_arguments={
            'workflow_file': workflow_file,
            'default_workflow': default_workflow,
            'execute_motion': execute_motion,
        }.items(),
        condition=IfCondition(start_workflow),
    )

    rs485_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_rs485'),
                'launch',
                'rs485_sensors.launch.py',
            ])
        ]),
        launch_arguments={
            'config_file': rs485_config_file,
            'start_laser': start_laser,
            'start_state_signal': start_state_signal,
            'laser_port': laser_port,
            'state_port': state_port,
            'state_auto_run_workflow': state_auto_run_workflow,
        }.items(),
    )

    io_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_io'),
                'launch',
                'gpio_io.launch.py',
            ])
        ]),
        launch_arguments={
            'start_io': start_io,
            'io_enabled': io_enabled,
            'io_backend': io_backend,
            'io_auto_run_workflow': io_auto_run_workflow,
            'io_allow_output_writes': io_allow_output_writes,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('workflow_file', default_value=default_workflow_file),
        DeclareLaunchArgument('default_workflow', default_value='cup_pick_place'),
        DeclareLaunchArgument('execute_motion', default_value='true'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('start_workflow', default_value='true'),
        DeclareLaunchArgument('start_laser', default_value='false'),
        DeclareLaunchArgument('start_state_signal', default_value='false'),
        DeclareLaunchArgument('start_io', default_value='false'),
        DeclareLaunchArgument('rs485_config_file', default_value=default_rs485_config_file),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('control_mode', default_value='position_velocity'),
        DeclareLaunchArgument('laser_port', default_value='/dev/ttyS4'),
        DeclareLaunchArgument('state_port', default_value='/dev/ttyS3'),
        DeclareLaunchArgument('state_auto_run_workflow', default_value='false'),
        DeclareLaunchArgument('io_enabled', default_value='false'),
        DeclareLaunchArgument('io_backend', default_value='sysfs'),
        DeclareLaunchArgument('io_auto_run_workflow', default_value='false'),
        DeclareLaunchArgument('io_allow_output_writes', default_value='false'),
        hardware_launch,
        workflow_launch,
        rs485_launch,
        io_launch,
    ])
