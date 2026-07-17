from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("execute_motion", default_value="true"),
        DeclareLaunchArgument("arm_group", default_value="arm"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("hand_frame", default_value="gripper_center"),
        Node(
            package="panthera_pose_tuner",
            executable="pose_tuner_node",
            name="panthera_pose_tuner",
            output="screen",
            parameters=[{
                "execute_motion": LaunchConfiguration("execute_motion"),
                "arm_group": LaunchConfiguration("arm_group"),
                "base_frame": LaunchConfiguration("base_frame"),
                "hand_frame": LaunchConfiguration("hand_frame"),
            }],
        ),
    ])
