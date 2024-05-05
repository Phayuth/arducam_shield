import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    cfg = os.path.join(get_package_share_directory("arducam_shield"), "config", "AR1820_MIPI_2Lane_RAW10_8b_1280x720.cfg")
    calibleft = os.path.join(get_package_share_directory("arducam_shield"), "calib_data", "stereo_left.yaml")
    calibright = os.path.join(get_package_share_directory("arducam_shield"), "calib_data", "stereo_right.yaml")

    la = []
    la.append(
        DeclareLaunchArgument(
            name="cfg_file",
            default_value=cfg,
            description="configuration file",
        ),
    )

    la.append(
        DeclareLaunchArgument(
            name="calib_file_left",
            default_value=calibleft,
            description="calibration file",
        ),
    )

    la.append(
        DeclareLaunchArgument(
            name="calib_file_right",
            default_value=calibright,
            description="calibration file",
        ),
    )

    ln = []
    ln.append(
        Node(
            package="arducam_shield",
            executable="arducam",
            name="arducam_node",
            namespace="camera_left",
            parameters=[
                {
                    "cfg_file": LaunchConfiguration("cfg_file"),
                    "calib_file": LaunchConfiguration("calib_file_left"),
                    "cam_index": 0,
                },
            ],
            output="screen",
        )
    )

    ln.append(
        Node(
            package="arducam_shield",
            executable="arducam",
            name="arducam_node",
            namespace="camera_right",
            parameters=[
                {
                    "cfg_file": LaunchConfiguration("cfg_file"),
                    "calib_file": LaunchConfiguration("calib_file_right"),
                    "cam_index": 1,
                },
            ],
            output="screen",
        )
    )

    return LaunchDescription(la + ln)
