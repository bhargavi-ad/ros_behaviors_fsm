import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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
        super().__init__('key_press_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel',10)
        self.settings = termios.tcgettattr(sys.stdin)
        self.timer = self.create_timer(0.1,self.get_key)
     
    
    def run_loop(self):
        """Check for keypress and send velocity"""
        msg = Twist()
        key = get_key(self.settings)
        

        if key == 'w':
             msg.linear.x = 0.2
        elif key == 's':
             msg.linear.x = -0.2
        elif key == 'a':
             msg.angular.z = 0.5
        elif key == 'd':
             msg.angular.z = -0.5
        elif key == '\x03':
             rclpy.shutdown()
             return
        
        self.pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = teleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 
