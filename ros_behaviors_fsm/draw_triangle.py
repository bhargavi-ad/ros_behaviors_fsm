import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from neato2_interfaces.msg import Bump 
import math


class drawTriNode(Node):
    """ A class that causes the Neato to drive in an equilateral triangle and stop when a bump sensor is activated"""
    def __init__(self):
        super().__init__('draw_triangle')
        # velpub publishes to the cmd_vel topic
        self.velpub = self.create_publisher(Twist, 'cmd_vel', 10)
        # every time something is published to the estop topic, the handle_estop function will run
        self.bump_state = False
        self.create_subscription(Bump, "bump", self.handle_bump, 10)
        # creates separate thread so while listening for estop, neato is making triangle
        self.run_triloop_thread = Thread(target=self.run_loop)
        self.run_triloop_thread.start()


    def handle_bump(self, msg):
        """Function to change the bump state variable based on whether any of the four Bump sensors have been activated
        
        Args: 
            msg: 0 or 1 from Bump class, represeting whether bump sensor has been activated (1) or not (0)
        """
        if msg.left_front == 1 or msg.right_front == 1 or msg.left_side == 1 or msg.right_side == 1:
            self.bump_state = True
    
    def drive(self, linear, angular):
        """ Publishes cmd_vel to Twist topic to set Neato linear and angular velocities
        Args: 
            linear: linear velocity in m/s
            angular: angular velocity in rad/s
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.velpub.publish(msg)
    
    def turn_tri(self):
        """Turns Neato 120 degrees based on set angular velocity as long as bump sensors are not activated """
        ang_vel = 0.3
        if self.bump_state == False:
            self.drive(linear= 0.0, angular= ang_vel)
            sleep(((math.pi/ang_vel)*(2/3)))
            self.drive(linear=0.0, angular=0.0)

    def drive_straight(self, distance):
        """ Drives Neato in straight line based on specified velocity as long as bump sensors are not activated 
        Args:
            distance: specified distance for each side of triangle (meters)
        
        """
        forward_vel = 0.1
        if self.bump_state == False:
            self.drive(linear=forward_vel, angular=0.0)
        sleep(distance/forward_vel)
        self.drive(linear=0.0, angular=0.0)



    def run_loop(self):
        """ Draws one triangle and terminates loop if any bump sensor is activated """
        sleep(1)
        
        for _ in range(3):
            if self.bump_state == False:
                print("driving forward")
                self.drive_straight(0.5)
            if self.bump_state == False:
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

