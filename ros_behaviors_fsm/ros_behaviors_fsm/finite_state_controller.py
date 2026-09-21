import rclpy
from rclpy.node import Node
from threading import Event
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import time
import math

class FollowTurnDrawNode(Node):
    """Follows the closest thing within 40 cm with the lidar, turns 180 degrees, then
    drives a square. Stops when estop is activated.

    States: SEARCH -> FOLLOW -> TURN -> DRAW -> DONE
    """
    def __init__(self):
        super().__init__('follow_turn_draw_with_estop')
        self.e_stop = Event()
        self.detect_radius = 0.4 #radius of detection
        self.follow_dist = 0.2 #how closely the neato should follow the person
        self.obj_dist = None #distance to the closest obj or none if nothing is detected
        self.obj_angle = None #angle to get to the closest obj detected

        self.state = 'SEARCH'
        self.search_vel = 0.1 #how fast to drive forward while looking for someone to follow
        self.run_time = 30.0 #the time we want it to follow the person for
        self.state_start = time.time() #when the current state began
        self.turn_vel = 0.3 #angular speed for the 180 degree turn

        turn_speed = 0.5 #angular speed for the 90 degree corners
        side = [(0.2, 0.0, 2.0), #drive forward 0.4 m: (linear, angular, seconds)
                (0.0, turn_speed, (math.pi / 2) / turn_speed)] #turn left 90 degrees
        self.draw_steps = side * 4 #four sides make a square
        self.step = 0 #which drawing step we are on

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)

        self.timer = self.create_timer(0.1, self.run_loop)

    def handle_estop(self, msg):
        """Handles messages received on the estop topic.

        Args:
            msg (std_msgs.msg.Bool): the message that takes value true if we
            estop and false otherwise.
        """
        if msg.data:
            self.e_stop.set()
            self.drive(linear=0.0, angular=0.0)

    def process_scan(self, msg):
        """Finds the closest reading within 0.4m and finds the dist and angle it is from neato"""
        distance = None
        angle = None

        for i, d in enumerate(msg.ranges):
            if d == 0.0 or d > self.detect_radius:
                continue #discard readings outside of the radius
            if distance == None or d < distance: #records first valid reading then keeps going
                distance = d #finds the closest obj and assign that to distance
                angle = i #gives the angle of that obj

        self.obj_dist = distance
        self.obj_angle = angle

    def set_state(self, new_state):
        """Switches to a new state and restarts the state timer."""
        self.get_logger().info(f'{self.state} -> {new_state}')
        self.state = new_state
        self.state_start = time.time()
        self.step = 0

    def run_loop(self):
        """Runs the state that we are currently in. The estop stops every state."""
        if self.e_stop.is_set(): #if the estop is activated by the bump
            return

        if self.state == 'SEARCH':
            self.search()
        elif self.state == 'FOLLOW':
            self.follow_person()
        elif self.state == 'TURN':
            self.turn_left()
        elif self.state == 'DRAW':
            self.draw_shape()
        else: #DONE
            self.drive(linear = 0.0, angular = 0.0)

    def search(self):
        """Drives forward until the lidar detects something to follow, then starts following.
        """
        if self.obj_dist is not None: #something is within the detection radius
            self.drive(linear = 0.0, angular = 0.0)
            self.set_state('FOLLOW') #the follow timer starts now
        else:
            self.drive(linear = self.search_vel, angular = 0.0)

    def follow_person(self):
        """Follows the person. Moves on to the turn after the 30 seconds are up.
        """
        if time.time() - self.state_start >= self.run_time: #if time is up
            self.drive(linear = 0.0, angular = 0.0)
            self.set_state('TURN') #hand off to the next state instead of shutting down
            return

        if self.obj_dist is None: #if it cannot find anything within the correct distance (just stops)
            self.drive(linear = 0.0, angular = 0.0)
            return

        angle = self.obj_angle
        if angle > 180:
            angle -= 360 #converts any angle greater than 180 into a negative by subtracting 360
        angular_vel = 0.01*angle #proportional turning speed (positive angle = left = counterclockwise)

        if self.obj_dist > self.follow_dist:
            linear_vel = 0.1 #keeps driving up to the obj
        else:
            linear_vel = 0.0
        self.drive(linear = linear_vel, angular = angular_vel)

    def turn_left(self):
        """Execute a 180 degree left turn (timed, so it doesn't block the timer loop)
        """
        if time.time() - self.state_start >= math.pi / self.turn_vel: #turned pi radians
            self.drive(linear = 0.0, angular = 0.0)
            self.set_state('DRAW')
        else:
            self.drive(linear = 0.0, angular = self.turn_vel)

    def draw_shape(self):
        """Steps through draw_steps one at a time to drive the square.
        state_start is reset at the start of every step.
        """
        if self.step >= len(self.draw_steps): #finished every side
            self.drive(linear = 0.0, angular = 0.0)
            self.set_state('DONE')
            return

        linear, angular, duration = self.draw_steps[self.step]
        if time.time() - self.state_start >= duration: #this step is finished
            self.step += 1
            self.state_start = time.time()
        else:
            self.drive(linear = linear, angular = angular)

    def drive(self, linear, angular):
        """Drive with the specified linear and angular velocity.

        Args:
            linear (_type_): the linear velocity in m/s
            angular (_type_): the angular velocity in radians/s
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)      # Initialize communication with ROS
    node = FollowTurnDrawNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shut down
    node.destroy_node()
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()
