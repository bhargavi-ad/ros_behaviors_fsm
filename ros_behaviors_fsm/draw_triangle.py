import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from neato2_interfaces.msg import Bump 
import math


class drawTriNode(Node):
    def __init__(self):
        super().__init__('draw_triangle')
        self.estop = Event()
        # velpub publishes to the cmd_vel topic
        self.velpub = self.create_publisher(Twist, 'cmd_vel', 10)
        # every time something is published to the estop topic, the handle_estop function will run
        self.create_subscription(Bump, "e_stop", self.handle_estop, 10)
        # creates separate thread so while listening for estop, neato is making triangle
        self.run_triloop_thread = Thread(target=self.run_triangle)
        self.run_triangle.start()


    def handle_estop(self, msg):
    # msg is the boolean value sent from the subscription to the estop topic
        if msg.data: # only runs if estop topic sends True for boolean val
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
            self.drive(linear= 0.0, angular= ang_vel)
            # continues to turn for x number of seconds that completes 120 degrees based on ang_vel
            sleep((math.pi*2)/(3*ang_vel))
            # stops neato at rotation spot
            self.drive(linear=0.0, angular=0.0)

    def drive_straight(self, distance):
        forward_vel = 0.1
        # if no estop, drive straight
        if not self.estop.is_set():
            self.drive(linear=forward_vel, angular=0.0)
            sleep(distance/forward_vel)
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
        print('done with run loop')

def main(args=None):
    rclpy.init(args=args)
    node = drawTriNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

