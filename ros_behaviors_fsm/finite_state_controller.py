import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
import tty
import select
import sys
import termios
from threading import Thread, Event
from time import sleep
from std_msgs.msg import Bool
from neato2_interfaces.msg import Bump
import math


class drawTriNode(Node):
    def __init__(self):
        super().__init__("draw_triangle")
        self.estop = Event()
        # velpub publishes to the cmd_vel topic
        self.velpub = self.create_publisher(Twist, "cmd_vel", 10)
        # every time something is published to the estop topic, the handle_estop function will run
        self.create_subscription(Bump, "e_stop", self.handle_estop, 10)
        # creates separate thread so while listening for estop, neato is making triangle
        self.run_triloop_thread = Thread(target=self.run_triangle)
        self.run_triangle.start()

    def handle_estop(self, msg):
        # msg is the boolean value sent from the subscription to the estop topic
        if msg.data:  # only runs if estop topic sends True for boolean val
            self.estop.set()
            self.drive(linear=0.0, angular=0.0)

    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.velpub.publish(msg)

    def turn_tri(self):
        ang_vel = 0.3
        if not self.estop.set():
            # as long as estop is false, only turn
            self.drive(linear=0.0, angular=ang_vel)
            # continues to turn for x number of seconds that completes 120 degrees based on ang_vel
            sleep((math.pi * 2) / (3 * ang_vel))
            # stops neato at rotation spot
            self.drive(linear=0.0, angular=0.0)

    def drive_straight(self, distance):
        forward_vel = 0.1
        # if no estop, drive straight
        if not self.estop.is_set():
            self.drive(linear=forward_vel, angular=0.0)
            sleep(distance / forward_vel)
            self.drive(linear=0.0, angular=0.0)

    def run_triangle(self):
        # callback function used to execute drawing a triangle when thread starts, keeps looping
        # until square finished or estop pressed
        self.drive(0.0, 0.0)
        sleep(1)
        for _ in range(3):
            if not self.estop.is_set():
                print("driving forward")
                self.drive_straight(0.5)
            if not self.estop.is_set():
                print("turning")
                self.turn_tri()
        print("done with run loop")


def get_key():
    """Grab key press from code sample"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


settings = termios.tcgetattr(sys.stdin)
key = None


class teleopNode(Node):
    """
    Class that publishes velocities for controlling Neato
    """

    def __init__(self):
        super().__init__("key_press_node")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.settings = termios.tcgettattr(sys.stdin)
        self.timer = self.create_timer(0.1, self.get_key)

    def run_loop(self):
        """Check for keypress and send velocity"""
        msg = Twist()
        key = get_key(self.settings)

        if key == "w":
            msg.linear.x = 0.2
        elif key == "s":
            msg.linear.x = -0.2
        elif key == "a":
            msg.angular.z = 0.5
        elif key == "d":
            msg.angular.z = -0.5
        elif key == "\x03":
            rclpy.shutdown()
            return

        self.pub.publish(msg)


class WallApproachNode(Node):
    """This class wraps the basic functionality of the node"""

    def __init__(self):
        super().__init__("wall_approach")
        # the run_loop adjusts the robot's velocity based on latest laser data
        self.create_timer(0.1, self.run_loop)
        self.create_subscription(
            LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data
        )
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # distance_to_obstacle is used to communciate laser data to run_loop
        self.distance_to_obstacle = None
        # Kp is the constant or to apply to the proportional error signal
        self.Kp = 0.4
        # target_distance is the desired distance to the obstacle in front
        self.target_distance = 1.2

    def run_loop(self):
        msg = Twist()
        if self.distance_to_obstacle is None:
            # if we haven't seen an obstacle yet, just go straight at fixed vel
            msg.linear.x = 0.1
        else:
            # use proportional control to set the velocity
            msg.linear.x = self.Kp * (self.distance_to_obstacle - self.target_distance)
        self.vel_pub.publish(msg)

    def process_scan(self, msg):
        if msg.ranges[0] != 0.0:
            # checking for the value 0.0 ensures the data is valid.
            self.distance_to_obstacle = msg.ranges[0]


def main(args=None):
    key = get_key(self.settings)

    if key == "t":
        # teleop
        rclpy.init(args=args)
        node = teleopNode()
        rclpy.spin(node)
        rclpy.shutdown()
    elif key == "r":
        # Triangle
        rclpy.init(args=args)
        node = drawTriNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    elif key == "y":
        # Wall follow
        rclpy.init(args=args)
        node = WallApproachNode()
        rclpy.spin(node)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
