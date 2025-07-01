import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Teleop keyboard control started')
        self.settings = termios.tcgetattr(sys.stdin)

    def run(self):
        try:
            print("Use WASD keys to move. Press Q to quit.")
            while True:
                key = self.get_key()
                if key == 'w':
                    self.move(0.2, 0.0)
                elif key == 's':
                    self.move(-0.2, 0.0)
                elif key == 'a':
                    self.move(0.0, 0.5)
                elif key == 'd':
                    self.move(0.0, -0.5)
                elif key == 'q':
                    break
                else:
                    self.move(0.0, 0.0)
        finally:
            self.move(0.0, 0.0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def move(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.2)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
