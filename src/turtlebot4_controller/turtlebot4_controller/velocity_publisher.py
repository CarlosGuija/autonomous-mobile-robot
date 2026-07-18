import signal

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')

        self.declare_parameter('linear_speed', 0.1)
        self.declare_parameter('angular_speed', 0.0)

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_unstamped',
            10,
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_velocity,
        )

    def publish_velocity(self):
        linear_speed = self.get_parameter('linear_speed').value
        angular_speed = self.get_parameter('angular_speed').value

        message = Twist()
        message.linear.x = float(linear_speed)
        message.angular.z = float(angular_speed)

        self.publisher.publish(message)

    def stop_robot(self):
        self.timer.cancel()

        stop_message = Twist()

        self.get_logger().info('Sending stop command...')

        for _ in range(20):
            self.publisher.publish(stop_message)
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

    node = VelocityPublisher()
    running = True

    def handle_sigint(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
