import math
import signal

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class RotateAngle(Node):

    def __init__(self):
        super().__init__('rotate_angle')

        self.declare_parameter('target_angle_degrees', 90.0)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('min_angular_speed', 0.1)
        self.declare_parameter('kp', 1.5)
        self.declare_parameter('tolerance', 0.01)

        target_angle_degrees = float(
            self.get_parameter('target_angle_degrees').value
        )
        self.target_angle = math.radians(target_angle_degrees)
        self.max_angular_speed = abs(
            float(self.get_parameter('max_angular_speed').value)
        )
        self.min_angular_speed = min(
            abs(float(self.get_parameter('min_angular_speed').value)),
            self.max_angular_speed,
        )
        self.kp = float(self.get_parameter('kp').value)
        self.tolerance = abs(
            float(self.get_parameter('tolerance').value)
        )

        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_unstamped',
            10,
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop,
        )

        self.previous_yaw = None
        self.accumulated_angle = 0.0
        self.finished = False

        self.get_logger().info(
            f'Rotating {target_angle_degrees:.2f} degrees'
        )

    def odom_callback(self, message):
        orientation = message.pose.pose.orientation

        siny_cosp = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y**2 + orientation.z**2
        )
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.previous_yaw is None:
            self.previous_yaw = current_yaw
            self.get_logger().info(
                f'Initial yaw: {current_yaw:.3f} rad'
            )
            return

        yaw_delta = current_yaw - self.previous_yaw
        yaw_delta = math.atan2(
            math.sin(yaw_delta),
            math.cos(yaw_delta),
        )

        self.accumulated_angle += yaw_delta
        self.previous_yaw = current_yaw

    def control_loop(self):
        if self.finished or self.previous_yaw is None:
            return

        angle_error = self.target_angle - self.accumulated_angle

        if abs(angle_error) <= self.tolerance:
            self.finished = True
            self.timer.cancel()
            self.publish_stop()

            self.get_logger().info(
                f'Target reached. '
                f'Rotation: {math.degrees(self.accumulated_angle):.2f} degrees'
            )
            return

        commanded_speed = self.kp * angle_error
        commanded_speed = max(
            -self.max_angular_speed,
            min(commanded_speed, self.max_angular_speed),
        )

        if abs(commanded_speed) < self.min_angular_speed:
            commanded_speed = math.copysign(
                self.min_angular_speed,
                angle_error,
            )

        command = Twist()
        command.linear.x = 0.0
        command.angular.z = commanded_speed

        self.velocity_publisher.publish(command)

        self.get_logger().info(
            f'Rotation: {math.degrees(self.accumulated_angle):.2f} deg | '
            f'Error: {math.degrees(angle_error):.2f} deg | '
            f'Speed: {commanded_speed:.3f} rad/s',
            throttle_duration_sec=0.5,
        )

    def publish_stop(self):
        stop_command = Twist()

        for _ in range(20):
            self.velocity_publisher.publish(stop_command)
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

    node = RotateAngle()
    running = True

    def handle_sigint(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        while running and rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.timer.cancel()
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
