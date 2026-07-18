import math
import signal

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class DriveDistance(Node):

    def __init__(self):
        super().__init__('drive_distance')

        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('max_speed', 0.2)
        self.declare_parameter('min_speed', 0.03)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('tolerance', 0.01)

        requested_distance = float(
            self.get_parameter('target_distance').value
        )

        self.direction = 1.0 if requested_distance >= 0.0 else -1.0
        self.target_distance = abs(requested_distance)

        self.max_speed = abs(
            float(self.get_parameter('max_speed').value)
        )
        self.min_speed = abs(
            float(self.get_parameter('min_speed').value)
        )
        self.kp = float(
            self.get_parameter('kp').value
        )
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

        self.initial_x = None
        self.initial_y = None
        self.current_x = None
        self.current_y = None

        self.finished = False

        direction_text = 'forward' if self.direction > 0.0 else 'backward'

        self.get_logger().info(
            f'Driving {direction_text} for {self.target_distance:.2f} m'
        )

    def odom_callback(self, message):
        self.current_x = message.pose.pose.position.x
        self.current_y = message.pose.pose.position.y

        if self.initial_x is None:
            self.initial_x = self.current_x
            self.initial_y = self.current_y

            self.get_logger().info(
                f'Initial position: '
                f'x={self.initial_x:.3f}, '
                f'y={self.initial_y:.3f}'
            )

    def control_loop(self):
        if self.finished:
            return

        if self.initial_x is None or self.current_x is None:
            return

        delta_x = self.current_x - self.initial_x
        delta_y = self.current_y - self.initial_y

        travelled_distance = math.sqrt(
            delta_x**2 + delta_y**2
        )

        distance_error = self.target_distance - travelled_distance

        if distance_error <= self.tolerance:
            self.finished = True
            self.timer.cancel()
            self.publish_stop()

            self.get_logger().info(
                f'Target reached. '
                f'Distance travelled: {travelled_distance:.3f} m'
            )
            return

        commanded_speed = self.kp * distance_error

        commanded_speed = min(
            commanded_speed,
            self.max_speed,
        )

        commanded_speed = max(
            commanded_speed,
            self.min_speed,
        )

        commanded_speed *= self.direction

        command = Twist()
        command.linear.x = commanded_speed
        command.angular.z = 0.0

        self.velocity_publisher.publish(command)

        self.get_logger().info(
            f'Distance: {travelled_distance:.3f} m | '
            f'Error: {distance_error:.3f} m | '
            f'Speed: {commanded_speed:.3f} m/s',
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

    node = DriveDistance()
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
