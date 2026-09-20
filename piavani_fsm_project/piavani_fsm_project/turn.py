import rclpy
from rclpy.node import Node
from threading import Thread, Event
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math

class Turn(Node):
    """A class that implements a node to pilot a robot in a square.
    """

    def __init__(self):
        super().__init__('draw_square_with_estop')
        self.e_stop = Event()
        # create a thread to handle long-running component
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)
        self.run_loop_thread = Thread(target=self.run_loop)
        self.run_loop_thread.start()


    def handle_estop(self, msg):
        """Handles messages received on the estop topic.

        Args:
            msg (std_msgs.msg.Bool): the message that takes value true if we
            estop and false otherwise.
        """ 
        if msg.data:
            self.e_stop.set()
            self.drive(linear=0.0, angular=0.0)

    def turn_left(self):
        """Execute a 180 degree left turn
        """
        angular_vel = 0.3
        if not self.e_stop.is_set():
            self.drive(linear=0.0, angular=angular_vel)
            sleep(math.pi / angular_vel)
            self.drive(linear=0.0, angular=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = Turn()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()