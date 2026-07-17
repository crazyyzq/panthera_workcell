from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            default_value='0.0.0.0',
            description='HTTP bind host for the Web HMI',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8080',
            description='HTTP port for the Web HMI',
        ),
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic published by the arm camera',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_raw',
            description='Depth image topic published by the arm camera',
        ),
        DeclareLaunchArgument(
            'compressed_rgb_topic',
            default_value='/camera/color/image_raw/compressed',
            description='Compressed RGB image topic used for faster Web HMI display',
        ),
        DeclareLaunchArgument(
            'subscribe_raw_rgb',
            default_value='true',
            description='Subscribe to raw RGB as a fallback when the compressed stream is stale',
        ),
        DeclareLaunchArgument(
            'camera_max_width',
            default_value='480',
            description='Maximum camera image width served to browsers',
        ),
        DeclareLaunchArgument(
            'camera_target_fps',
            default_value='30.0',
            description='Maximum HMI camera conversion/refresh rate',
        ),
        DeclareLaunchArgument(
            'camera_worker_threads',
            default_value='3',
            description='Background worker threads used for raw camera preview conversion',
        ),
        DeclareLaunchArgument(
            'tool_pose_base_frame',
            default_value='base_link',
            description='Base frame used for end-effector pose display',
        ),
        DeclareLaunchArgument(
            'tool_pose_frame',
            default_value='gripper_center',
            description='Tool frame used for end-effector pose display',
        ),
        DeclareLaunchArgument(
            'tool_pose_rate_hz',
            default_value='10.0',
            description='Refresh rate for end-effector TF display',
        ),
        DeclareLaunchArgument(
            'tool_pose_max_jump_m',
            default_value='0.08',
            description='Single-frame tool pose jump threshold rejected by HMI display filter',
        ),
        DeclareLaunchArgument(
            'tool_pose_max_quat_jump',
            default_value='0.25',
            description='Single-frame quaternion jump threshold rejected by HMI display filter',
        ),
        DeclareLaunchArgument(
            'tool_pose_filter_accept_after_count',
            default_value='3',
            description='Consecutive outlier samples required before HMI accepts a new tool pose',
        ),
        Node(
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
        ),
    ])
