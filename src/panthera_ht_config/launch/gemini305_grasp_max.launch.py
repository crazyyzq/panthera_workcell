from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    color_width = LaunchConfiguration("color_width")
    color_height = LaunchConfiguration("color_height")
    color_fps = LaunchConfiguration("color_fps")
    color_format = LaunchConfiguration("color_format")
    depth_width = LaunchConfiguration("depth_width")
    depth_height = LaunchConfiguration("depth_height")
    depth_fps = LaunchConfiguration("depth_fps")
    depth_format = LaunchConfiguration("depth_format")
    depth_registration = LaunchConfiguration("depth_registration")
    align_mode = LaunchConfiguration("align_mode")
    align_target_stream = LaunchConfiguration("align_target_stream")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    enable_frame_sync = LaunchConfiguration("enable_frame_sync")

    orbbec_launch = PathJoinSubstitution([
        FindPackageShare("orbbec_camera"),
        "launch",
        "gemini305.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("color_width", default_value="1280"),
        DeclareLaunchArgument("color_height", default_value="800"),
        DeclareLaunchArgument("color_fps", default_value="30"),
        DeclareLaunchArgument(
            "color_format",
            default_value="YUYV",
            description=(
                "Gemini305 color stream format. 1280x800@30 supports YUYV; "
                "using YUYV avoids MJPG decode stalls seen on the tested host."
            ),
        ),
        DeclareLaunchArgument("depth_width", default_value="1280"),
        DeclareLaunchArgument("depth_height", default_value="800"),
        DeclareLaunchArgument("depth_fps", default_value="30"),
        DeclareLaunchArgument("depth_format", default_value="Y16"),
        DeclareLaunchArgument(
            "depth_registration",
            default_value="false",
            description=(
                "Enable depth-to-color registration. Keep this false for "
                "Gemini305 1280x800@30 because hardware D2C is not supported "
                "by that profile on the tested driver."
            ),
        ),
        DeclareLaunchArgument(
            "align_mode",
            default_value="SW",
            description="Depth alignment mode passed to the Orbbec launch when registration is enabled.",
        ),
        DeclareLaunchArgument("align_target_stream", default_value="COLOR"),
        DeclareLaunchArgument(
            "enable_point_cloud",
            default_value="false",
            description="Disable point cloud by default for HMI use to reduce USB/CPU load.",
        ),
        DeclareLaunchArgument(
            "enable_frame_sync",
            default_value="false",
            description=(
                "Disable Orbbec RGB/depth frame synchronization by default for "
                "1280x800 HMI streaming. This keeps both streams free-running and "
                "avoids color-stream stalls seen with sync enabled on the tested host."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(orbbec_launch),
            launch_arguments={
                "depth_registration": depth_registration,
                "align_mode": align_mode,
                "align_target_stream": align_target_stream,
                "color_width": color_width,
                "color_height": color_height,
                "color_fps": color_fps,
                "color_format": color_format,
                "depth_width": depth_width,
                "depth_height": depth_height,
                "depth_fps": depth_fps,
                "depth_format": depth_format,
                "enable_point_cloud": enable_point_cloud,
                "enable_frame_sync": enable_frame_sync,
            }.items(),
        )
    ])
