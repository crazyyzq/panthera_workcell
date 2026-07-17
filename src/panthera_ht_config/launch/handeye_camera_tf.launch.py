""" Static transform publisher acquired via easy_handeye2 """
""" EYE-IN-HAND: gripper_center -> camera_color_optical_frame """

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="handeye_camera_tf_publisher",
            output="screen",
            arguments=[
                "--frame-id", "gripper_center",
                "--child-frame-id", "camera_color_optical_frame",

                "--x", "-0.087791",
                "--y", "0.009808",
                "--z", "0.072285",

                "--qx", "-0.583228",
                "--qy", "0.564567",
                "--qz", "-0.406011",
                "--qw", "0.419839",
            ],
        )
    ])