"""Launch the deterministic headless benchmark as a ROS-installed executable."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("robot_control_benchmark")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=[share, "/config/benchmark_quick.yaml"]
            ),
            DeclareLaunchArgument("output", default_value="/tmp/robot_control_benchmark"),
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "robot_control_benchmark", "benchmark",
                    "--config", LaunchConfiguration("config"),
                    "--output", LaunchConfiguration("output"),
                ],
                output="screen",
            ),
        ]
    )

