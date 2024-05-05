import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # AR1820_MIPI_2Lane_RAW10_8b_1920x1080
    # AR1820_MIPI_4Lane_RAW10_8b_1280x720
    cfg = os.path.join(get_package_share_directory("arducam_shield"), "config", "AR1820_MIPI_2Lane_RAW10_8b_1280x720.cfg")
    calib = os.path.join(get_package_share_directory("arducam_shield"), "calib_data", "mono_1920x1080.yaml")

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
            name="calib_file",
            default_value=calib,
            description="calibration file",
        ),
    )

    ln = []
    ln.append(
        Node(
            package="arducam_shield",
            executable="arducam",
            name="arducam_node",
            namespace="camera_1",
            parameters=[
                {
                    "cfg_file": LaunchConfiguration("cfg_file"),
                    "calib_file": LaunchConfiguration("calib_file"),
                },
            ],
            output="screen",
        )
    )

    return LaunchDescription(la + ln)
