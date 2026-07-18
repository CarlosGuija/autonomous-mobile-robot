"""Coordinate custom motion actions to move a TurtleBot4 in a square."""

from enum import auto, Enum
import math
import signal

from action_msgs.msg import GoalStatus
from irobot_create_msgs.action import Undock
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from turtlebot4_interfaces.action import DriveDistance
from turtlebot4_interfaces.action import RotateAngle


class MotionState(Enum):
    """States used by the square motion controller."""

    WAITING_FOR_SERVERS = auto()
    UNDOCK = auto()
    DRIVE_SIDE = auto()
    ROTATE = auto()
    FINISHED = auto()
    FAILED = auto()


class MoveSquare(Node):
    """Drive a square using undock and custom motion Action Servers."""

    def __init__(self):
        """Initialize parameters, state, and asynchronous action clients."""
        super().__init__('move_square')

        self.declare_parameter('side_length', 1.0)
        self.declare_parameter('turn_angle_degrees', 90.0)
        self.declare_parameter('number_of_sides', 4)
        self.declare_parameter('max_translation_speed', 0.2)
        self.declare_parameter('max_rotation_speed', 0.5)

        self.side_length = float(
            self.get_parameter('side_length').value
        )
        self.turn_angle_degrees = float(
            self.get_parameter('turn_angle_degrees').value
        )
        self.number_of_sides = int(
            self.get_parameter('number_of_sides').value
        )
        self.max_translation_speed = float(
            self.get_parameter('max_translation_speed').value
        )
        self.max_rotation_speed = float(
            self.get_parameter('max_rotation_speed').value
        )

        self.state = None
        self.completed_sides = 0
        self.active_goal_handle = None
        self.goal_request_pending = False
        self.shutdown_requested = False
        self.server_timer = None

        self.drive_client = None
        self.rotate_client = None
        self.undock_client = None

        self.transition_to(MotionState.WAITING_FOR_SERVERS)

        if not self.validate_parameters():
            return

        self.drive_client = ActionClient(
            self,
            DriveDistance,
            '/custom_drive_distance',
        )
        self.rotate_client = ActionClient(
            self,
            RotateAngle,
            '/custom_rotate_angle',
        )
        self.undock_client = ActionClient(
            self,
            Undock,
            '/undock',
        )

        self.server_timer = self.create_timer(
            0.5,
            self.check_action_servers,
        )

    def transition_to(self, new_state):
        """Change state and log every transition."""
        old_state = self.state.name if self.state is not None else 'STARTUP'
        self.state = new_state
        self.get_logger().info(
            f'State transition: {old_state} -> {new_state.name}'
        )

    def validate_parameters(self):
        """Validate all motion parameters before contacting action servers."""
        errors = []

        if not math.isfinite(self.side_length):
            errors.append('side_length must be finite')
        elif self.side_length <= 0.0:
            errors.append('side_length must be greater than zero')
        if self.number_of_sides <= 0:
            errors.append('number_of_sides must be greater than zero')
        if not math.isfinite(self.max_translation_speed):
            errors.append('max_translation_speed must be finite')
        elif self.max_translation_speed <= 0.0:
            errors.append(
                'max_translation_speed must be greater than zero'
            )
        if not math.isfinite(self.max_rotation_speed):
            errors.append('max_rotation_speed must be finite')
        elif self.max_rotation_speed <= 0.0:
            errors.append('max_rotation_speed must be greater than zero')
        if not math.isfinite(self.turn_angle_degrees):
            errors.append('turn_angle_degrees must be finite')
        elif self.turn_angle_degrees == 0.0:
            errors.append('turn_angle_degrees must not be zero')

        if errors:
            self.fail('Invalid parameters: ' + '; '.join(errors))
            return False

        return True

    def check_action_servers(self):
        """Wait asynchronously for action servers and safe startup state."""
        if self.shutdown_requested:
            return

        drive_ready = self.drive_client.server_is_ready()
        rotate_ready = self.rotate_client.server_is_ready()
        undock_ready = self.undock_client.server_is_ready()

        if not drive_ready or not rotate_ready or not undock_ready:
            self.get_logger().info(
                'Waiting for /undock, /custom_drive_distance, and '
                '/custom_rotate_angle...',
                throttle_duration_sec=2.0,
            )
            return

        self.server_timer.cancel()
        self.transition_to(MotionState.UNDOCK)
        self.send_undock_goal()

    def send_undock_goal(self):
        """Send one asynchronous undock goal."""
        if not self.reserve_goal_request('undock'):
            return
        self.goal_request_pending = True
        send_future = self.undock_client.send_goal_async(Undock.Goal())
        send_future.add_done_callback(self.undock_goal_response_callback)

    def undock_goal_response_callback(self, future):
        """Check undock goal acceptance and request its result."""
        self.goal_request_pending = False

        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: B902
            if self.shutdown_requested:
                rclpy.try_shutdown()
            else:
                self.fail(f'Failed to send undock goal: {error}')
            return

        if not goal_handle.accepted:
            if self.shutdown_requested:
                self.get_logger().info(
                    'Undock goal was rejected during shutdown'
                )
                rclpy.try_shutdown()
            else:
                self.fail('Undock goal was rejected')
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info('Undock goal accepted')

        if self.shutdown_requested:
            self.cancel_active_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        """Check the undock result before starting the first side."""
        self.active_goal_handle = None

        if self.shutdown_requested:
            rclpy.try_shutdown()
            return

        try:
            wrapped_result = future.result()
        except Exception as error:  # noqa: B902
            self.fail(f'Failed to get undock result: {error}')
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.fail(
                'Undock failed with action status '
                f'{self.status_name(wrapped_result.status)}'
            )
            return

        if wrapped_result.result.is_docked:
            self.fail('Undock action succeeded but robot is still docked')
            return

        self.get_logger().info('Robot undocked successfully')
        self.transition_to(MotionState.DRIVE_SIDE)
        self.send_drive_goal(self.side_length, 'side drive')

    def send_drive_goal(self, distance, operation):
        """Send one asynchronous drive goal."""
        if not self.reserve_goal_request(operation):
            return
        goal = DriveDistance.Goal()
        goal.distance = distance
        goal.max_speed = self.max_translation_speed

        self.goal_request_pending = True
        send_future = self.drive_client.send_goal_async(
            goal,
            feedback_callback=self.drive_feedback_callback,
        )
        send_future.add_done_callback(
            lambda future: self.drive_goal_response_callback(
                future,
                operation,
            )
        )

    def drive_goal_response_callback(self, future, operation):
        """Check drive goal acceptance and request its result."""
        self.goal_request_pending = False

        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: B902
            if self.shutdown_requested:
                rclpy.try_shutdown()
            else:
                self.fail(f'Failed to send {operation} goal: {error}')
            return

        if not goal_handle.accepted:
            if self.shutdown_requested:
                self.get_logger().info(
                    f'{operation.capitalize()} goal was rejected during '
                    'shutdown'
                )
                rclpy.try_shutdown()
            else:
                self.fail(f'{operation.capitalize()} goal was rejected')
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info(
            f'{operation.capitalize()} goal accepted'
        )

        if self.shutdown_requested:
            self.cancel_active_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.drive_result_callback(result, operation)
        )

    def drive_feedback_callback(self, feedback_message):
        """Log concise remaining-distance feedback."""
        remaining = feedback_message.feedback.remaining_distance
        self.get_logger().info(
            f'Remaining distance: {remaining:.3f} m',
            throttle_duration_sec=0.5,
        )

    def drive_result_callback(self, future, operation):
        """Check a drive result and advance the state machine."""
        self.active_goal_handle = None

        if self.shutdown_requested:
            rclpy.try_shutdown()
            return

        try:
            wrapped_result = future.result()
        except Exception as error:  # noqa: B902
            self.fail(f'Failed to get {operation} result: {error}')
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.fail(
                f'{operation.capitalize()} failed with action status '
                f'{self.status_name(wrapped_result.status)}'
            )
            return

        if not wrapped_result.result.success:
            self.fail(
                f'{operation.capitalize()} reported failure: '
                f'{wrapped_result.result.message}'
            )
            return

        self.completed_sides += 1
        self.get_logger().info(
            f'Completed sides: {self.completed_sides}/'
            f'{self.number_of_sides}'
        )
        self.transition_to(MotionState.ROTATE)
        self.send_rotate_goal(self.turn_angle_degrees, 'square turn')

    def send_rotate_goal(self, angle_degrees, operation):
        """Send one asynchronous relative rotation goal."""
        if not self.reserve_goal_request(operation):
            return
        goal = RotateAngle.Goal()
        goal.angle_degrees = angle_degrees
        goal.max_angular_speed = self.max_rotation_speed

        self.goal_request_pending = True
        send_future = self.rotate_client.send_goal_async(
            goal,
            feedback_callback=self.rotate_feedback_callback,
        )
        send_future.add_done_callback(
            lambda future: self.rotate_goal_response_callback(
                future,
                operation,
            )
        )

    def rotate_goal_response_callback(self, future, operation):
        """Check rotation goal acceptance and request its result."""
        self.goal_request_pending = False

        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: B902
            if self.shutdown_requested:
                rclpy.try_shutdown()
            else:
                self.fail(f'Failed to send {operation} goal: {error}')
            return

        if not goal_handle.accepted:
            if self.shutdown_requested:
                self.get_logger().info(
                    f'{operation.capitalize()} goal was rejected during '
                    'shutdown'
                )
                rclpy.try_shutdown()
            else:
                self.fail(f'{operation.capitalize()} goal was rejected')
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info(
            f'{operation.capitalize()} goal accepted'
        )

        if self.shutdown_requested:
            self.cancel_active_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.rotate_result_callback(result, operation)
        )

    def rotate_feedback_callback(self, feedback_message):
        """Log concise remaining-angle feedback."""
        remaining = feedback_message.feedback.remaining_angle_degrees
        self.get_logger().info(
            f'Remaining angle: {remaining:.2f} deg',
            throttle_duration_sec=0.5,
        )

    def rotate_result_callback(self, future, operation):
        """Check a rotation result and advance the state machine."""
        self.active_goal_handle = None

        if self.shutdown_requested:
            rclpy.try_shutdown()
            return

        try:
            wrapped_result = future.result()
        except Exception as error:  # noqa: B902
            self.fail(f'Failed to get {operation} result: {error}')
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.fail(
                f'{operation.capitalize()} failed with action status '
                f'{self.status_name(wrapped_result.status)}'
            )
            return

        if not wrapped_result.result.success:
            self.fail(
                f'{operation.capitalize()} reported failure: '
                f'{wrapped_result.result.message}'
            )
            return

        if self.completed_sides == self.number_of_sides:
            self.transition_to(MotionState.FINISHED)
            self.get_logger().info('Square motion completed successfully')
            rclpy.try_shutdown()
            return

        self.transition_to(MotionState.DRIVE_SIDE)
        self.send_drive_goal(self.side_length, 'side drive')

    def reserve_goal_request(self, operation):
        """Prevent this client from ever overlapping action goals."""
        if self.goal_request_pending or self.active_goal_handle is not None:
            self.fail(
                f'Internal goal overlap prevented while starting {operation}'
            )
            return False
        return True

    def fail(self, message):
        """Log an error, enter FAILED, and stop the node."""
        self.get_logger().error(message)
        if self.state != MotionState.FAILED:
            self.transition_to(MotionState.FAILED)
        rclpy.try_shutdown()

    def request_shutdown(self):
        """Handle Ctrl+C without synchronously waiting on an action."""
        if self.shutdown_requested:
            return

        self.shutdown_requested = True
        self.get_logger().info('Shutdown requested')

        if self.server_timer is not None:
            self.server_timer.cancel()

        if self.active_goal_handle is not None:
            self.cancel_active_goal()
        elif self.goal_request_pending:
            self.get_logger().info(
                'Waiting for pending goal response before cancellation'
            )
        else:
            rclpy.try_shutdown()

    def cancel_active_goal(self):
        """Request cancellation of the active action goal."""
        self.get_logger().info('Canceling active goal')
        cancel_future = self.active_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        """Finish shutdown after an asynchronous cancellation request."""
        try:
            response = future.result()
            if response.goals_canceling:
                self.get_logger().info('Active goal cancellation accepted')
            else:
                self.get_logger().warning(
                    'Action server did not accept goal cancellation'
                )
        except Exception as error:  # noqa: B902
            self.get_logger().error(
                f'Failed to cancel active goal: {error}'
            )
        finally:
            rclpy.try_shutdown()

    @staticmethod
    def status_name(status):
        """Return a readable action status name."""
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, f'UNKNOWN({status})')


def main(args=None):
    """Run the move-square action coordinator."""
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

    node = MoveSquare()

    def handle_sigint(signum, frame):
        node.request_shutdown()

    previous_sigint_handler = signal.signal(
        signal.SIGINT,
        handle_sigint,
    )

    try:
        if rclpy.ok():
            rclpy.spin(node)
    finally:
        signal.signal(signal.SIGINT, previous_sigint_handler)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
