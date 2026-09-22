import rclpy
from rclpy.node import Node
from threading import Event
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from neato2_interfaces.msg import Bump
import time
import math

class FollowTurnDrawNode(Node):
    """Follows the closest thing within 40 cm with the lidar, turns 180 degrees, then
    drives a pentagon. Stops when estop is activated.

    States: SEARCH -> FOLLOW -> TURN -> DRAW -> DONE
    """
    def __init__(self):
        super().__init__('follow_turn_draw_with_estop')
        self.e_stop = Event()
        self.detect_radius = 0.6 #detection radius
        self.follow_dist = 0.2 #person to neato dist
        self.min_valid_dist = 0.07 #discard readings this close
        self.obj_dist = None #distance to the closest obj or none if nothing is detected
        self.obj_angle = None #angle to get to the closest obj detected
        self.state = 'SEARCH'
        self.search_vel = 0.1 #speed when searching for person
        self.run_time = 30.0 #following time
        self.state_start = time.time() #when the current state began
        self.turn_vel = 0.3 #speed for 180

        self.k_linear = 0.8 #speed scales with distance past follow_dist
        self.max_linear_vel = 0.3 #max speed when following
        self.last_seen = time.time() #last time person seen
        self.last_angle = 0 #last angle person was seen at
        self.lost_grace = 0.7 #how long to track last known angle

        turn_speed = 0.5 #speed for the corners
        side = [(0.2, 0.0, 2.0), #sides of 0.4 m
                (0.0, turn_speed, (2 * math.pi / 5) / turn_speed)] #turn left 72 degrees
        self.draw_steps = side * 5
        self.step = 0 #drawing step

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Bool, 'estop', self.handle_estop, 10)
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)
        self.create_subscription(Bump, 'bump', self.process_bump, 10)

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

    def process_bump(self, msg):
        """ """
        if msg.left_front or msg.right_front or msg.left_side or msg.right_side:
             self.e_stop.set()
             self.drive(linear=0.0, angular=0.0)

    def process_scan(self, msg):
        """Finds the closest reading within 0.4m and finds the dist and angle it is from neato"""
        distance = None
        angle = None

        for i, d in enumerate(msg.ranges):
            if d == 0.0 or d > self.detect_radius or d < self.min_valid_dist:
                continue #discard readings outside of the radius
            if distance == None or d < distance: #records first valid reading then keeps going
                distance = d #finds the closest obj and assign that to distance
                angle = i #gives the angle of that obj

        self.obj_dist = distance
        self.obj_angle = angle
        if distance is not None:
            self.last_seen = time.time()
            self.last_angle = angle

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
        if self.obj_dist is not None: #something is within 0.6 radius
            self.drive(linear = 0.0, angular = 0.0)
            self.set_state('FOLLOW') #start timer for following
        else:
            self.drive(linear = self.search_vel, angular = 0.0)

    def follow_person(self):
        """Follows the person. Moves on to the turn after the 30 seconds are up.
        """
        if time.time() - self.state_start >= self.run_time: #check time
            self.drive(linear = 0.0, angular = 0.0)
            self.set_state('TURN') #move to turn state
            return

        if self.obj_dist is None: #if person lost
            if time.time() - self.last_seen < self.lost_grace: #turn to last known angle if in grace period
                angle = self.last_angle
                if angle > 180:
                    angle -= 360
                angular_vel = 0.01*angle
                self.drive(linear = 0.0, angular = angular_vel)
            else: #stop past grace period
                self.drive(linear = 0.0, angular = 0.0)
            return

        angle = self.obj_angle
        if angle > 180:
            angle -= 360
        angular_vel = 0.01*angle #proportional turning speed

        #proportional speed scales with distance
        linear_vel = min(self.max_linear_vel, max(0.0, self.k_linear*(self.obj_dist - self.follow_dist)))
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
        if self.step >= len(self.draw_steps): #finished all sides
            self.drive(linear = 0.0, angular = 0.0)
            self.set_state('DONE')
            return

        linear, angular, duration = self.draw_steps[self.step]
        if time.time() - self.state_start >= duration:
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
