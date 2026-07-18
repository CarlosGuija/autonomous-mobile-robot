"""Custom odometry-based rotate-angle Action Server."""

import math
import signal
import threading

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from turtlebot4_interfaces.action import RotateAngle


class RotateAngleActionServer(Node):
    """Rotate through a signed relative angle using accumulated odometry."""

    STOP_COMMAND_COUNT = 20

    def __init__(self):
        """Initialize control parameters, ROS entities, and goal state."""
        super().__init__('rotate_angle_action_server')

        self.declare_parameter('kp', 1.5)
        self.declare_parameter('min_angular_speed', 0.1)
        self.declare_parameter('tolerance_degrees', 1.0)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('odom_timeout', 1.0)

        self.kp = float(self.get_parameter('kp').value)
        self.min_angular_speed = float(
            self.get_parameter('min_angular_speed').value
        )
        self.tolerance_degrees = float(
            self.get_parameter('tolerance_degrees').value
        )
        self.tolerance = math.radians(self.tolerance_degrees)
        self.control_period = float(
            self.get_parameter('control_period').value
        )
        self.odom_timeout = float(
            self.get_parameter('odom_timeout').value
        )
        self.parameters_valid = self._validate_parameters()

        self._lock = threading.Lock()
        self._goal_reserved = False
        self._active_goal_handle = None
        self._completion_future = None
        self._terminal_kind = None
        self._terminal_result = None
        self._stop_commands_remaining = 0
        self._finalizing = False

        self._odom_valid = False
        self._last_odom_time = None
        self._latest_yaw = 0.0

        self._previous_yaw = None
        self._accumulated_angle = 0.0
        self._target_angle = 0.0
        self._max_angular_speed = 0.0

        # The execute coroutine only awaits a Future. Separate callback groups
        # plus a multithreaded executor keep odometry, control, and cancel
        # callbacks runnable while that Future is pending.
        self._odom_group = MutuallyExclusiveCallbackGroup()
        self._control_group = MutuallyExclusiveCallbackGroup()
        self._action_group = ReentrantCallbackGroup()

        self._velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_unstamped',
            10,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10,
            callback_group=self._odom_group,
        )
        self._control_timer = self.create_timer(
            self.control_period,
            self._control_loop,
            callback_group=self._control_group,
        )
        self._action_server = ActionServer(
            self,
            RotateAngle,
            '/custom_rotate_angle',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_group,
        )

        # This server only arbitrates rotate goals. A separate external client
        # could still overlap a custom drive goal on /cmd_vel_unstamped.
        self.get_logger().info('Custom rotate-angle Action Server is ready')

    def _validate_parameters(self):
        """Return whether all controller parameters are usable."""
        errors = []
        positive_parameters = (
            ('kp', self.kp),
            ('tolerance_degrees', self.tolerance_degrees),
            ('control_period', self.control_period),
            ('odom_timeout', self.odom_timeout),
        )
        for name, value in positive_parameters:
            if not math.isfinite(value) or value <= 0.0:
                errors.append(f'{name} must be finite and greater than zero')
        if (
            not math.isfinite(self.min_angular_speed)
            or self.min_angular_speed < 0.0
        ):
            errors.append(
                'min_angular_speed must be finite and non-negative'
            )

        if errors:
            self.get_logger().error(
                'Invalid parameters: ' + '; '.join(errors)
            )
            return False
        return True

    def _odom_callback(self, message):
        """Validate odometry and accumulate normalized yaw increments."""
        orientation = message.pose.pose.orientation
        values = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        now = self.get_clock().now()

        if not all(math.isfinite(value) for value in values):
            with self._lock:
                self._last_odom_time = now
                self._odom_valid = False
            return

        norm = math.sqrt(sum(value * value for value in values))
        if norm <= 1e-12:
            with self._lock:
                self._last_odom_time = now
                self._odom_valid = False
            return

        x = orientation.x / norm
        y = orientation.y / norm
        z = orientation.z / norm
        w = orientation.w / norm
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        with self._lock:
            self._last_odom_time = now
            self._latest_yaw = yaw
            self._odom_valid = True

            if (
                self._active_goal_handle is not None
                and self._terminal_kind is None
                and self._previous_yaw is not None
            ):
                yaw_delta = yaw - self._previous_yaw
                yaw_delta = math.atan2(
                    math.sin(yaw_delta),
                    math.cos(yaw_delta),
                )
                self._accumulated_angle += yaw_delta
                self._previous_yaw = yaw

    def _odom_is_fresh_locked(self, now):
        """Check odometry age using the node clock for simulated time."""
        if not self._odom_valid or self._last_odom_time is None:
            return False
        age = (now - self._last_odom_time).nanoseconds / 1e9
        return 0.0 <= age <= self.odom_timeout

    def _goal_callback(self, goal_request):
        """Validate and reserve the server before accepting a goal."""
        now = self.get_clock().now()
        reason = None

        with self._lock:
            if not self.parameters_valid:
                reason = 'controller parameters are invalid'
            elif self._goal_reserved:
                reason = 'another rotate goal is already active'
            elif not math.isfinite(goal_request.angle_degrees):
                reason = 'angle_degrees must be finite'
            elif (
                not math.isfinite(goal_request.max_angular_speed)
                or goal_request.max_angular_speed <= 0.0
            ):
                reason = (
                    'max_angular_speed must be finite and greater than zero'
                )
            elif not self._odom_is_fresh_locked(now):
                reason = 'valid, recent odometry is unavailable'
            else:
                self._goal_reserved = True

        if reason is not None:
            self.get_logger().warning(f'Rejecting rotate goal: {reason}')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepting rotate goal: {goal_request.angle_degrees:.2f} deg'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accept cancellation and stop publishing movement immediately."""
        with self._lock:
            if not self._goal_reserved or self._finalizing:
                return CancelResponse.REJECT

            if (
                self._active_goal_handle is not None
                and goal_handle is not self._active_goal_handle
            ):
                return CancelResponse.REJECT

            if self._active_goal_handle is not None:
                result = self._make_result_locked(
                    False,
                    'Rotate goal canceled',
                )
                self._begin_terminal_locked('canceled', result, force=True)

            self._publish_stop_locked()

        self.get_logger().info('Rotate goal cancellation accepted')
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        """Initialize a goal and await timer-driven completion."""
        completion_future = Future()
        goal = goal_handle.request
        now = self.get_clock().now()

        with self._lock:
            self._active_goal_handle = goal_handle
            self._completion_future = completion_future
            self._terminal_kind = None
            self._terminal_result = None
            self._stop_commands_remaining = 0
            self._finalizing = False

            self._previous_yaw = self._latest_yaw
            self._accumulated_angle = 0.0
            self._target_angle = math.radians(float(goal.angle_degrees))
            self._max_angular_speed = float(goal.max_angular_speed)

            if goal_handle.is_cancel_requested:
                result = self._make_result_locked(
                    False,
                    'Rotate goal canceled before execution',
                )
                self._begin_terminal_locked('canceled', result)
            elif not self._odom_is_fresh_locked(now):
                result = self._make_result_locked(
                    False,
                    'Rotate aborted: valid odometry is unavailable',
                )
                self._begin_terminal_locked('aborted', result)
            elif self._target_angle == 0.0:
                result = self._make_result_locked(
                    True,
                    'Zero-angle goal completed',
                )
                self._begin_terminal_locked('succeeded', result)

        return await completion_future

    def _control_loop(self):
        """Run one non-blocking controller or safe-stop iteration."""
        finalize_data = None
        feedback_data = None

        with self._lock:
            goal_handle = self._active_goal_handle
            if goal_handle is None:
                return

            if self._terminal_kind is not None:
                if self._stop_commands_remaining > 0:
                    self._publish_stop_locked()
                    self._stop_commands_remaining -= 1
                if (
                    self._stop_commands_remaining == 0
                    and not self._finalizing
                ):
                    self._finalizing = True
                    finalize_data = (
                        goal_handle,
                        self._completion_future,
                        self._terminal_kind,
                        self._terminal_result,
                    )
            elif goal_handle.is_cancel_requested:
                result = self._make_result_locked(
                    False,
                    'Rotate goal canceled',
                )
                self._begin_terminal_locked('canceled', result)
                self._publish_stop_locked()
                self._stop_commands_remaining -= 1
            else:
                now = self.get_clock().now()
                if not self._odom_is_fresh_locked(now):
                    result = self._make_result_locked(
                        False,
                        'Rotate aborted: odometry became unavailable',
                    )
                    self._begin_terminal_locked('aborted', result)
                    self._publish_stop_locked()
                    self._stop_commands_remaining -= 1
                else:
                    feedback_data = self._run_controller_locked(goal_handle)

        if feedback_data is not None:
            handle, feedback = feedback_data
            try:
                handle.publish_feedback(feedback)
            except Exception as error:  # noqa: B902
                self._abort_from_exception(
                    f'Could not publish rotate feedback: {error}'
                )

        if finalize_data is not None:
            self._finalize_goal(*finalize_data)

    def _run_controller_locked(self, goal_handle):
        """Calculate and publish one rotation command with the lock held."""
        angle_error = self._target_angle - self._accumulated_angle

        if abs(angle_error) <= self.tolerance:
            result = self._make_result_locked(
                True,
                'Target angle reached',
            )
            self._begin_terminal_locked('succeeded', result)
            self._publish_stop_locked()
            self._stop_commands_remaining -= 1
            return None

        speed = self.kp * angle_error
        speed = max(
            -self._max_angular_speed,
            min(speed, self._max_angular_speed),
        )
        minimum = min(self.min_angular_speed, self._max_angular_speed)
        if abs(speed) < minimum:
            speed = math.copysign(minimum, angle_error)

        command = Twist()
        command.angular.z = speed
        self._velocity_publisher.publish(command)

        feedback = RotateAngle.Feedback()
        feedback.remaining_angle_degrees = math.degrees(angle_error)
        feedback.current_angular_speed = speed
        return goal_handle, feedback

    def _make_result_locked(self, success, message):
        """Build a rotate result from the accumulated relative angle."""
        result = RotateAngle.Result()
        result.success = success
        result.message = message
        result.angle_rotated_degrees = math.degrees(
            self._accumulated_angle
        )
        return result

    def _begin_terminal_locked(self, kind, result, force=False):
        """Enter safe stopping before publishing an action result."""
        if self._terminal_kind is not None and not force:
            return
        self._terminal_kind = kind
        self._terminal_result = result
        self._stop_commands_remaining = self.STOP_COMMAND_COUNT

    def _publish_stop_locked(self):
        """Publish one zero velocity command."""
        self._velocity_publisher.publish(Twist())

    def _abort_from_exception(self, message):
        """Convert an unexpected controller exception into an abort."""
        self.get_logger().error(message)
        with self._lock:
            if self._active_goal_handle is None or self._finalizing:
                return
            result = self._make_result_locked(False, message)
            self._begin_terminal_locked('aborted', result, force=True)
            self._publish_stop_locked()

    def _finalize_goal(self, goal_handle, future, kind, result):
        """Set terminal action state after all stop commands were sent."""
        try:
            if kind == 'succeeded':
                goal_handle.succeed()
            elif kind == 'canceled':
                goal_handle.canceled()
            else:
                goal_handle.abort()
        except Exception as error:  # noqa: B902
            self.get_logger().error(f'Failed to finalize rotate goal: {error}')

        with self._lock:
            self._active_goal_handle = None
            self._completion_future = None
            self._terminal_kind = None
            self._terminal_result = None
            self._stop_commands_remaining = 0
            self._finalizing = False
            self._goal_reserved = False

        if future is not None and not future.done():
            future.set_result(result)

        log = self.get_logger().info
        if kind == 'aborted':
            log = self.get_logger().error
        log(result.message)

    def stop_for_shutdown(self):
        """Stop safely and resolve an active goal before node destruction."""
        finalize_data = None
        with self._lock:
            for _ in range(self.STOP_COMMAND_COUNT):
                self._publish_stop_locked()

            if self._active_goal_handle is not None:
                result = self._make_result_locked(
                    False,
                    'Rotate aborted because the server is shutting down',
                )
                finalize_data = (
                    self._active_goal_handle,
                    self._completion_future,
                    'aborted',
                    result,
                )
                self._finalizing = True
            elif self._goal_reserved:
                self._goal_reserved = False

        if finalize_data is not None:
            self._finalize_goal(*finalize_data)


def main(args=None):
    """Run the rotate-angle server with concurrent callback execution."""
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = RotateAngleActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    running = True

    def handle_sigint(signum, frame):
        nonlocal running
        running = False

    previous_sigint_handler = signal.signal(signal.SIGINT, handle_sigint)

    try:
        while running and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        signal.signal(signal.SIGINT, previous_sigint_handler)
        node.stop_for_shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
