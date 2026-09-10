"""Gazebo Harmonic demo with differential-drive plant and selected controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    benchmark_share = get_package_share_directory("robot_control_benchmark")
    gz_share = get_package_share_directory("ros_gz_sim")
    resource_path = os.path.join(benchmark_share, "models")
    prior_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    world = os.path.join(benchmark_share, "worlds", "benchmark_world.sdf")
    config = os.path.join(benchmark_share, "config", "ros_controller.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("controller", default_value="mpc"),
            DeclareLaunchArgument("trajectory", default_value="figure_eight"),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                resource_path + (os.pathsep + prior_path if prior_path else ""),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(gz_share, "launch", "gz_sim.launch.py")),
                launch_arguments={"gz_args": f"-r {world}"}.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                ],
                output="screen",
            ),
            Node(
                package="robot_control_benchmark",
                executable="controller_node",
                name="trajectory_controller",
                parameters=[
                    config,
                    {
                        "controller": LaunchConfiguration("controller"),
                        "trajectory": LaunchConfiguration("trajectory"),
                    },
                ],
                output="screen",
            ),
        ]
    )

