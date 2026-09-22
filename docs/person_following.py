import rclpy
from rclpy.node import Node
from threading import Event
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import time

class PersonFollowingNode(Node):
    """Follows the closest thing within 40 cm with the lidar and stops when estop is activated"""
    def __init__(self):
        super().__init__('follow_person_with_estop')
        self.e_stop = Event()
        self.detect_radius = 0.4 #radius of detection
        self.follow_dist = 0.2 #how closely the neato should follow the person
        self.obj_dist = None #distance to the closest obj or none if nothing is detected
        self.obj_angle = None #angle to get to the closest obj detected

        self.state = 'FOLLOW' #current FSM state: FOLLOW -> TURN -> DRAW -> DONE
        self.run_time = 30.0 #the time we want it to follow the person for
        self.start_time = time.time() #starts the timer

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


    def run_loop(self):
        """Executes the main logic for following the person.
        Stops running after the 30 seconds are up.
        """
        if self.state != 'FOLLOW': #following is finished, later states handle driving
            return

        if time.time() - self.start_time >= self.run_time: #if time is up
            self.drive(linear = 0.0, angular = 0.0)
            self.state = 'TURN' #hand off to the next state instead of shutting down
            return

        if self.e_stop.is_set(): #if the estop is activated by the bump
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
    node = PersonFollowingNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()