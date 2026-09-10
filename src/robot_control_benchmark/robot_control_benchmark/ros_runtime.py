"""ROS 2 runtime implementation kept separate from the dependency-free core."""

from __future__ import annotations

import math

from .controllers import create
from .simulator import select_reference
from .trajectories import generate
from .types import State


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def run(args=None) -> None:
    try:
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Odometry, Path
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile
    except ImportError as exc:  # pragma: no cover
        raise SystemExit("ROS dependencies unavailable; source /opt/ros/jazzy/setup.bash") from exc

    class TrackingNode(Node):
        def __init__(self) -> None:
            super().__init__("trajectory_controller")
            self.declare_parameter("controller", "mpc")
            self.declare_parameter("trajectory", "figure_eight")
            self.declare_parameter("control_frequency", 20.0)
            self.declare_parameter("odom_topic", "/odom")
            self.declare_parameter("cmd_vel_topic", "/cmd_vel")
            controller_name = self.get_parameter("controller").value
            trajectory_name = self.get_parameter("trajectory").value
            frequency = float(self.get_parameter("control_frequency").value)
            if frequency <= 0.0:
                raise ValueError("control_frequency must be positive")
            self._dt = 1.0 / frequency
            self._controller = create(controller_name)
            self._trajectory = generate(trajectory_name, self._dt)
            self._state = None
            self._index = 0
            self._cycles = 0
            self._deadline_misses = 0
            self._cmd_pub = self.create_publisher(
                Twist, self.get_parameter("cmd_vel_topic").value, 10
            )
            self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._path_pub = self.create_publisher(Path, "/reference_path", path_qos)
            self.create_subscription(
                Odometry, self.get_parameter("odom_topic").value, self._odom_callback, 20
            )
            self.create_timer(self._dt, self._control_callback)
            self._publish_path(Path, PoseStamped)
            self.get_logger().info(
                f"tracking {trajectory_name} with {controller_name} at {frequency:.1f} Hz"
            )

        def _publish_path(self, path_type, pose_type) -> None:
            message = path_type()
            message.header.frame_id = "odom"
            message.header.stamp = self.get_clock().now().to_msg()
            for x, y, yaw in zip(
                self._trajectory.x, self._trajectory.y, self._trajectory.yaw, strict=True
            ):
                pose = pose_type()
                pose.header = message.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
                pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
                message.poses.append(pose)
            self._path_pub.publish(message)

        def _odom_callback(self, message) -> None:
            pose = message.pose.pose
            self._state = State(
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ),
            )

        def _control_callback(self) -> None:
            started = self.get_clock().now()
            if self._state is None:
                self._diagnostic(1, "waiting for odometry", 0.0)
                return
            self._index = select_reference(self._state, self._trajectory, self._index)
            if self._index >= len(self._trajectory.time) - 1:
                self._cmd_pub.publish(Twist())
                self._diagnostic(0, "trajectory complete", 0.0)
                return
            command = self._controller.compute(
                self._state, self._trajectory, self._index, self._dt
            )
            message = Twist()
            message.linear.x = command.linear
            message.angular.z = command.angular
            self._cmd_pub.publish(message)
            elapsed = (self.get_clock().now() - started).nanoseconds * 1e-9
            self._cycles += 1
            self._deadline_misses += int(elapsed > self._dt)
            self._diagnostic(0, "tracking", elapsed)

        def _diagnostic(self, level: int, summary: str, elapsed: float) -> None:
            array = DiagnosticArray()
            array.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.name = "robot_control_benchmark/controller"
            status.hardware_id = "simulation"
            status.level = bytes((level,))
            status.message = summary
            status.values = [
                KeyValue(key="controller", value=self._controller.name),
                KeyValue(key="trajectory", value=self._trajectory.name),
                KeyValue(key="reference_index", value=str(self._index)),
                KeyValue(key="compute_ms", value=f"{elapsed * 1e3:.4f}"),
                KeyValue(key="cycles", value=str(self._cycles)),
                KeyValue(key="deadline_misses", value=str(self._deadline_misses)),
            ]
            array.status.append(status)
            self._diag_pub.publish(array)

    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
