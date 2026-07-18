"""Custom odometry-based drive-distance Action Server."""

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
from turtlebot4_interfaces.action import DriveDistance


class DriveDistanceActionServer(Node):
    """Drive a signed distance using odometry and proportional control."""

    STOP_COMMAND_COUNT = 20

    def __init__(self):
        """Initialize control parameters, ROS entities, and goal state."""
        super().__init__('drive_distance_action_server')

        self.declare_parameter('kp', 0.8)
        self.declare_parameter('min_speed', 0.03)
        self.declare_parameter('tolerance', 0.01)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('odom_timeout', 1.0)

        self.kp = float(self.get_parameter('kp').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.tolerance = float(self.get_parameter('tolerance').value)
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
        self._current_x = 0.0
        self._current_y = 0.0

        self._start_x = 0.0
        self._start_y = 0.0
        self._target_distance = 0.0
        self._direction = 1.0
        self._max_speed = 0.0
        self._distance_travelled = 0.0

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
            DriveDistance,
            '/custom_drive_distance',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._action_group,
        )

        # This server only arbitrates drive goals. A separate external client
        # could still overlap a custom rotate goal on /cmd_vel_unstamped.
        self.get_logger().info(
            'Custom drive-distance Action Server is ready'
        )

    def _validate_parameters(self):
        """Return whether all controller parameters are usable."""
        errors = []
        positive_parameters = (
            ('kp', self.kp),
            ('tolerance', self.tolerance),
            ('control_period', self.control_period),
            ('odom_timeout', self.odom_timeout),
        )
        for name, value in positive_parameters:
            if not math.isfinite(value) or value <= 0.0:
                errors.append(f'{name} must be finite and greater than zero')
        if not math.isfinite(self.min_speed) or self.min_speed < 0.0:
            errors.append('min_speed must be finite and non-negative')

        if errors:
            self.get_logger().error(
                'Invalid parameters: ' + '; '.join(errors)
            )
            return False
        return True

    def _odom_callback(self, message):
        """Store the latest valid planar odometry measurement."""
        x = message.pose.pose.position.x
        y = message.pose.pose.position.y
        now = self.get_clock().now()

        with self._lock:
            self._last_odom_time = now
            if math.isfinite(x) and math.isfinite(y):
                self._current_x = x
                self._current_y = y
                self._odom_valid = True
            else:
                self._odom_valid = False

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
                reason = 'another drive goal is already active'
            elif not math.isfinite(goal_request.distance):
                reason = 'distance must be finite'
            elif (
                not math.isfinite(goal_request.max_speed)
                or goal_request.max_speed <= 0.0
            ):
                reason = 'max_speed must be finite and greater than zero'
            elif not self._odom_is_fresh_locked(now):
                reason = 'valid, recent odometry is unavailable'
            else:
                self._goal_reserved = True

        if reason is not None:
            self.get_logger().warning(f'Rejecting drive goal: {reason}')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepting drive goal: {goal_request.distance:.3f} m'
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
                    'Drive goal canceled',
                )
                self._begin_terminal_locked('canceled', result, force=True)

            self._publish_stop_locked()

        self.get_logger().info('Drive goal cancellation accepted')
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

            self._start_x = self._current_x
            self._start_y = self._current_y
            self._target_distance = abs(float(goal.distance))
            self._direction = 1.0 if goal.distance >= 0.0 else -1.0
            self._max_speed = float(goal.max_speed)
            self._distance_travelled = 0.0

            if goal_handle.is_cancel_requested:
                result = self._make_result_locked(
                    False,
                    'Drive goal canceled before execution',
                )
                self._begin_terminal_locked('canceled', result)
            elif not self._odom_is_fresh_locked(now):
                result = self._make_result_locked(
                    False,
                    'Drive aborted: valid odometry is unavailable',
                )
                self._begin_terminal_locked('aborted', result)
            elif self._target_distance == 0.0:
                result = self._make_result_locked(
                    True,
                    'Zero-distance goal completed',
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
                    'Drive goal canceled',
                )
                self._begin_terminal_locked('canceled', result)
                self._publish_stop_locked()
                self._stop_commands_remaining -= 1
            else:
                now = self.get_clock().now()
                if not self._odom_is_fresh_locked(now):
                    result = self._make_result_locked(
                        False,
                        'Drive aborted: odometry became unavailable',
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
                    f'Could not publish drive feedback: {error}'
                )

        if finalize_data is not None:
            self._finalize_goal(*finalize_data)

    def _run_controller_locked(self, goal_handle):
        """Calculate and publish one drive command with the lock held."""
        delta_x = self._current_x - self._start_x
        delta_y = self._current_y - self._start_y
        self._distance_travelled = math.hypot(delta_x, delta_y)
        remaining = self._target_distance - self._distance_travelled

        if remaining <= self.tolerance:
            result = self._make_result_locked(
                True,
                'Target distance reached',
            )
            self._begin_terminal_locked('succeeded', result)
            self._publish_stop_locked()
            self._stop_commands_remaining -= 1
            return None

        speed = min(self.kp * remaining, self._max_speed)
        speed = max(speed, min(self.min_speed, self._max_speed))
        signed_speed = self._direction * speed

        command = Twist()
        command.linear.x = signed_speed
        self._velocity_publisher.publish(command)

        feedback = DriveDistance.Feedback()
        feedback.remaining_distance = self._direction * remaining
        feedback.current_speed = signed_speed
        return goal_handle, feedback

    def _make_result_locked(self, success, message):
        """Build a drive result from the latest controller state."""
        result = DriveDistance.Result()
        result.success = success
        result.message = message
        result.distance_travelled = (
            self._direction * self._distance_travelled
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
            self.get_logger().error(f'Failed to finalize drive goal: {error}')

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
                    'Drive aborted because the server is shutting down',
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
    """Run the drive-distance server with concurrent callback execution."""
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = DriveDistanceActionServer()
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
