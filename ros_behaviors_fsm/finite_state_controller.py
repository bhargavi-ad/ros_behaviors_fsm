import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from enum import Enum

# Import the individual behavior classes
from teleop import teleopNode
from draw_triangle import drawTriNode
from wall_follower import WallApproachNode


class RobotState(Enum):
    """Enumeration of possible robot states"""

    IDLE = "idle"
    TELEOP = "teleop"
    DRAW_TRIANGLE = "draw_triangle"
    WALL_FOLLOW = "wall_follow"
    EMERGENCY_STOP = "emergency_stop"


class FiniteStateController(Node):
    """
    Finite State Machine controller that coordinates the three robot behaviors:
    1. Teleop (manual control)
    2. Draw Triangle (autonomous triangle drawing)
    3. Wall Follow (obstacle avoidance)
    """

    def __init__(self):
        super().__init__("finite_state_controller")

        # State management
        self.current_state = RobotState.IDLE
        self.previous_state = None

        # Behavior instances
        self.teleop_behavior = None
        self.triangle_behavior = None
        self.wall_follow_behavior = None

        # Emergency stop handling
        self.emergency_stop = False
        self.create_subscription(Bump, "bump", self.handle_bump, 10)

        # Velocity publisher for emergency stop
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Main control loop
        self.create_timer(0.1, self.run_fsm)

    def handle_bump(self, msg):
        """Handle bump sensor activation - emergency stop"""
        if (
            msg.left_front == 1
            or msg.right_front == 1
            or msg.left_side == 1
            or msg.right_side == 1
        ):
            self.emergency_stop = True
            self.transition_to_state(RobotState.EMERGENCY_STOP)
            self.get_logger().warn("Emergency stop activated!")

    def transition_to_state(self, new_state):
        """Transition to a new state, cleaning up previous state"""
        if self.current_state == new_state:
            return

        self.previous_state = self.current_state
        self.current_state = new_state

        # Stop current behavior
        self.stop_current_behavior()

        # Start new behavior
        self.start_behavior_for_state(new_state)

        self.get_logger().info(
            f"State transition: {self.previous_state.value} -> {self.current_state.value}"
        )

    def stop_current_behavior(self):
        """Stop the currently active behavior"""
        if self.teleop_behavior:
            self.teleop_behavior.destroy_node()
            self.teleop_behavior = None
        if self.triangle_behavior:
            self.triangle_behavior.destroy_node()
            self.triangle_behavior = None
        if self.wall_follow_behavior:
            self.wall_follow_behavior.destroy_node()
            self.wall_follow_behavior = None

        # Stop robot movement
        self.stop_robot()

    def start_behavior_for_state(self, state):
        """Start the appropriate behavior for the given state"""
        if state == RobotState.TELEOP:
            self.teleop_behavior = teleopNode()
            self.get_logger().info("Started Teleop behavior")

        elif state == RobotState.DRAW_TRIANGLE:
            self.triangle_behavior = drawTriNode()
            self.get_logger().info("Started Draw Triangle behavior")

        elif state == RobotState.WALL_FOLLOW:
            self.wall_follow_behavior = WallApproachNode()
            self.get_logger().info("Started Wall Follow behavior")

        elif state == RobotState.EMERGENCY_STOP:
            self.stop_robot()
            self.get_logger().warn("Emergency stop - all behaviors stopped")

    def stop_robot(self):
        """Immediately stop robot movement"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

    def run_fsm(self):
        """Main finite state machine loop"""
        # Check for emergency stop
        if self.emergency_stop and self.current_state != RobotState.EMERGENCY_STOP:
            self.transition_to_state(RobotState.EMERGENCY_STOP)
            return

        # Reset emergency stop if no longer active
        if self.emergency_stop and self.current_state == RobotState.EMERGENCY_STOP:
            self.emergency_stop = False
            self.transition_to_state(RobotState.IDLE)
            return

        # Spin the active behavior
        if self.teleop_behavior:
            rclpy.spin_once(self.teleop_behavior, timeout_sec=0.001)
        elif self.triangle_behavior:
            rclpy.spin_once(self.triangle_behavior, timeout_sec=0.001)
        elif self.wall_follow_behavior:
            rclpy.spin_once(self.wall_follow_behavior, timeout_sec=0.001)

    def set_state(self, new_state):
        """Public method to set the FSM state from external code"""
        self.transition_to_state(new_state)

    def get_current_state(self):
        """Get the current state of the FSM"""
        return self.current_state

    def is_emergency_stopped(self):
        """Check if the robot is in emergency stop state"""
        return self.emergency_stop


def main(args=None):
    rclpy.init(args=args)

    try:
        controller = FiniteStateController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
