import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data


class WallApproachNode(Node):
    """Wall following behavior using proportional control"""

    def __init__(self):
        super().__init__("wall_approach")

        # Timer for control loop
        self.create_timer(0.1, self.run_loop)

        # Laser scan subscription
        self.create_subscription(
            LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data
        )

        # Velocity publisher
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Control parameters
        self.distance_to_obstacle = None
        self.Kp = 0.4  # Proportional gain
        self.target_distance = 1.2  # Desired distance to wall (meters)

        # Safety limits
        self.max_linear_vel = 0.3  # Maximum linear velocity
        self.min_linear_vel = -0.1  # Maximum reverse velocity
        self.max_angular_vel = 0.5  # Maximum angular velocity

    def run_loop(self):
        """Main control loop for wall following"""
        msg = Twist()

        if self.distance_to_obstacle is None:
            # No obstacle detected, move forward at moderate speed
            msg.linear.x = 0.15
            msg.angular.z = 0.0
        else:
            # Calculate error and apply proportional control
            error = self.distance_to_obstacle - self.target_distance
            linear_vel = self.Kp * error

            # Apply safety limits
            linear_vel = max(self.min_linear_vel, min(self.max_linear_vel, linear_vel))

            msg.linear.x = linear_vel
            msg.angular.z = 0.0

            # Log current behavior
            self.get_logger().debug(
                f"Distance: {self.distance_to_obstacle:.2f}m, "
                f"Target: {self.target_distance:.2f}m, "
                f"Error: {error:.2f}m, "
                f"Velocity: {linear_vel:.2f}m/s"
            )

        self.vel_pub.publish(msg)

    def process_scan(self, msg):
        """Process laser scan data to find obstacle distance"""
        if msg.ranges[0] != 0.0:
            # Check for valid data (non-zero range)
            self.distance_to_obstacle = msg.ranges[0]
        else:
            # Invalid data, keep previous reading
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WallApproachNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
