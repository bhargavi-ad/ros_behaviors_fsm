import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math


class drawTriNode(Node):
    def __init__(self):
        super().__init__('draw triangle')
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.run_loop)
        self.turns_executed = 0
        self.executing_turn = False
        self.side_length = 0.5      # the length in meters of a square side
        self.time_per_side = 5.0    # duration in seconds to drive the square side
        self.time_per_turn = 2.0    # duration in seconds to turn 90 degrees
        # start_time_of_segment indicates when a particular part of the square was
        # started (e.g., a straight segment or a turn)
        self.start_time_of_segment = None  

    def run_loop(self):
        if self.start_time_of_segment is None:
            self.start_time_of_segment = self.get_clock().now()
        msg = Twist()
        if self.executing_turn:
            segment_duration = self.time_per_turn
        else:
            segment_duration = self.time_per_side
        if self.get_clock().now() - self.start_time_of_segment > rclpy.time.Duration(seconds=segment_duration):
            if self.executing_turn:
                self.turns_executed += 1
            self.executing_turn = not self.executing_turn
            self.start_time_of_segment = None
            print(self.executing_turn, self.turns_executed)
        else:
            if self.executing_turn:
                msg.angular.z = (math.pi / 2) / segment_duration
            else:
                msg.linear.x = self.side_length / segment_duration
        self.vel_pub.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = drawTriNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()        