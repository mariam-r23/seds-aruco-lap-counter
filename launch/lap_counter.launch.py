"""Launch the ArUco lap counter.

Same launch-file pattern as urc_perception/launch/perception.launch.py
(LaunchDescription + launch_ros), but starts our own node instead of
aruco_ros so the only dependencies are OpenCV + cv_bridge.

    ros2 launch aruco_lap_counter lap_counter.launch.py \
        image_topic:=/camera/image_raw gate_sequence:="[0,1,2,3]"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("aruco_dictionary", default_value="DICT_4X4_50"),
        DeclareLaunchArgument("publish_debug_image", default_value="false"),
        Node(
            package="aruco_lap_counter",
            executable="lap_counter_node",
            name="aruco_lap_counter",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "aruco_dictionary": LaunchConfiguration("aruco_dictionary"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                "gate_sequence": [0, 1, 2, 3],
                "confirm_frames": 3,
                "cooldown_s": 2.0,
                "min_side_px": 40.0,
            }],
        ),
    ])
