from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
    simulation = LaunchConfiguration('simulation')
    start_cell = LaunchConfiguration('start_cell')
    start_camera = LaunchConfiguration('start_camera')
    start_workflow = LaunchConfiguration('start_workflow')
    start_laser = LaunchConfiguration('start_laser')
    start_pose_tuner = LaunchConfiguration('start_pose_tuner')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    compressed_rgb_topic = LaunchConfiguration('compressed_rgb_topic')
    subscribe_raw_rgb = LaunchConfiguration('subscribe_raw_rgb')
    camera_max_width = LaunchConfiguration('camera_max_width')
    camera_target_fps = LaunchConfiguration('camera_target_fps')
    camera_worker_threads = LaunchConfiguration('camera_worker_threads')
    tool_pose_base_frame = LaunchConfiguration('tool_pose_base_frame')
    tool_pose_frame = LaunchConfiguration('tool_pose_frame')
    tool_pose_rate_hz = LaunchConfiguration('tool_pose_rate_hz')
    tool_pose_max_jump_m = LaunchConfiguration('tool_pose_max_jump_m')
    tool_pose_max_quat_jump = LaunchConfiguration('tool_pose_max_quat_jump')
    tool_pose_filter_accept_after_count = LaunchConfiguration('tool_pose_filter_accept_after_count')
    camera_color_width = LaunchConfiguration('camera_color_width')
    camera_color_height = LaunchConfiguration('camera_color_height')
    camera_depth_width = LaunchConfiguration('camera_depth_width')
    camera_depth_height = LaunchConfiguration('camera_depth_height')
    camera_color_fps = LaunchConfiguration('camera_color_fps')
    camera_depth_fps = LaunchConfiguration('camera_depth_fps')
    camera_color_format = LaunchConfiguration('camera_color_format')
    camera_depth_format = LaunchConfiguration('camera_depth_format')
    camera_depth_registration = LaunchConfiguration('camera_depth_registration')
    camera_align_mode = LaunchConfiguration('camera_align_mode')
    camera_align_target_stream = LaunchConfiguration('camera_align_target_stream')
    camera_enable_point_cloud = LaunchConfiguration('camera_enable_point_cloud')
    camera_enable_frame_sync = LaunchConfiguration('camera_enable_frame_sync')
    rs485_config_file = LaunchConfiguration('rs485_config_file')
    laser_port = LaunchConfiguration('laser_port')
    laser_offset_mm = LaunchConfiguration('laser_offset_mm')

    default_rs485_config_file = PathJoinSubstitution([
        FindPackageShare('panthera_rs485'),
        'config',
        'rs485_devices.yaml',
    ])

    cell_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_spectrometer_cell'),
                'launch',
                'spectrometer_cell.launch.py',
            ])
        ]),
        launch_arguments={
            'simulation': simulation,
        }.items(),
        condition=IfCondition(start_cell),
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_ht_config'),
                'launch',
                'gemini305_grasp_max.launch.py',
            ])
        ]),
        launch_arguments={
            'color_width': camera_color_width,
            'color_height': camera_color_height,
            'color_fps': camera_color_fps,
            'color_format': camera_color_format,
            'depth_width': camera_depth_width,
            'depth_height': camera_depth_height,
            'depth_fps': camera_depth_fps,
            'depth_format': camera_depth_format,
            'depth_registration': camera_depth_registration,
            'align_mode': camera_align_mode,
            'align_target_stream': camera_align_target_stream,
            'enable_point_cloud': camera_enable_point_cloud,
            'enable_frame_sync': camera_enable_frame_sync,
        }.items(),
        condition=IfCondition(start_camera),
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
            'execute_motion': 'true',
            'default_workflow': 'fixed_large_motion_demo',
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
            'start_state_signal': 'false',
            'laser_port': laser_port,
            'laser_offset_mm': laser_offset_mm,
        }.items(),
    )

    pose_tuner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('panthera_pose_tuner'),
                'launch',
                'pose_tuner.launch.py',
            ])
        ]),
        launch_arguments={
            'execute_motion': 'true',
        }.items(),
        condition=IfCondition(start_pose_tuner),
    )

    web_node = Node(
        package='panthera_web_hmi',
        executable='web_hmi_node',
        name='panthera_web_hmi',
        output='screen',
        parameters=[{
            'host': host,
            'port': ParameterValue(port, value_type=int),
            'rgb_topic': rgb_topic,
            'depth_topic': depth_topic,
            'compressed_rgb_topic': compressed_rgb_topic,
            'subscribe_raw_rgb': ParameterValue(subscribe_raw_rgb, value_type=bool),
            'camera_max_width': ParameterValue(camera_max_width, value_type=int),
            'camera_target_fps': ParameterValue(camera_target_fps, value_type=float),
            'camera_worker_threads': ParameterValue(camera_worker_threads, value_type=int),
            'tool_pose_base_frame': tool_pose_base_frame,
            'tool_pose_frame': tool_pose_frame,
            'tool_pose_rate_hz': ParameterValue(tool_pose_rate_hz, value_type=float),
            'tool_pose_max_jump_m': ParameterValue(tool_pose_max_jump_m, value_type=float),
            'tool_pose_max_quat_jump': ParameterValue(tool_pose_max_quat_jump, value_type=float),
            'tool_pose_filter_accept_after_count': ParameterValue(
                tool_pose_filter_accept_after_count,
                value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('start_cell', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('start_workflow', default_value='false'),
        DeclareLaunchArgument('start_laser', default_value='false'),
        DeclareLaunchArgument('start_pose_tuner', default_value='false'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('compressed_rgb_topic', default_value='/camera/color/image_raw/compressed'),
        DeclareLaunchArgument('subscribe_raw_rgb', default_value='true'),
        DeclareLaunchArgument('camera_max_width', default_value='480'),
        DeclareLaunchArgument('camera_target_fps', default_value='30.0'),
        DeclareLaunchArgument('camera_worker_threads', default_value='3'),
        DeclareLaunchArgument('tool_pose_base_frame', default_value='base_link'),
        DeclareLaunchArgument('tool_pose_frame', default_value='gripper_center'),
        DeclareLaunchArgument('tool_pose_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('tool_pose_max_jump_m', default_value='0.08'),
        DeclareLaunchArgument('tool_pose_max_quat_jump', default_value='0.25'),
        DeclareLaunchArgument('tool_pose_filter_accept_after_count', default_value='3'),
        DeclareLaunchArgument('camera_color_width', default_value='1280'),
        DeclareLaunchArgument('camera_color_height', default_value='800'),
        DeclareLaunchArgument('camera_depth_width', default_value='1280'),
        DeclareLaunchArgument('camera_depth_height', default_value='800'),
        DeclareLaunchArgument('camera_color_fps', default_value='30'),
        DeclareLaunchArgument('camera_depth_fps', default_value='30'),
        DeclareLaunchArgument('camera_color_format', default_value='YUYV'),
        DeclareLaunchArgument('camera_depth_format', default_value='Y16'),
        DeclareLaunchArgument('camera_depth_registration', default_value='false'),
        DeclareLaunchArgument('camera_align_mode', default_value='SW'),
        DeclareLaunchArgument('camera_align_target_stream', default_value='COLOR'),
        DeclareLaunchArgument('camera_enable_point_cloud', default_value='false'),
        DeclareLaunchArgument('camera_enable_frame_sync', default_value='false'),
        DeclareLaunchArgument('rs485_config_file', default_value=default_rs485_config_file),
        DeclareLaunchArgument('laser_port', default_value='/dev/ttyS4'),
        DeclareLaunchArgument('laser_offset_mm', default_value='0.0'),
        cell_launch,
        camera_launch,
        workflow_launch,
        rs485_launch,
        pose_tuner_launch,
        web_node,
    ])
