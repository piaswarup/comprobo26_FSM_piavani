# RoboBehaviors and Finite State Machines Project
Author Names: Avani Patil and Pia Swarup

For Olin ENGR3590 Computational Introduction to Robotics

The goal of this project is to program a Neato robot to execute a set of behaviors organized into a finite state machine. The programmed behaviors are following a person for thirty seconds, executing a 180 degree turn, and drawing a pentagon. The implementation includes a single finite state machine, FollowTurnDrawNode, which links the behaviors together. The Neato starts by searching for a person to follow within a set distance. It then follows that person using LIDAR data for thirty seconds. After thirty seconds, a 180 degree turn is executed. Finally, the Neato traces out a pentagon. An emergency stop is implemented with the Bump sensor on the Neato which stops the robot at any point in the sequence. 

The States:

SEARCH - The Neato drives at a low speed until the LIDAR detects an object within a 0.6m radius to follow

FOLLOW - After an object is detected, the robot tracks and follows it while maintaining a following distance 0.2m for thirty seconds. 

TURN - The Neato executes a 180 degree turn in place

DRAW - The Neato drives in a pentagon trajectory, using time to dictate when to turn

DONE - The robot stops and remains idle

Key design choices include a timer-driven state machine that runs “run_loop” every 0.1 seconds, a top-level estop check, and proportional control for following. A timer-driven state machine was used instead of implementing the sleep function to not block the estop method and keep track of time for the time dependent Follow and Draw states. The estop check at the beginning of the run loop allows the estop to take priority over any other state without needing to tediously implement the estop in every behavior. Proportional control was used over the bang-bang control method to produce smoother tracking rather than a fixed-speed approach. 

A major takeaway from this project was the difference in the physical Neato versus the simulator. Sensor noise, timing, and physical dynamics on the real robot didn't always match what was seen on the simulator. Certain parameters like the person detection radius had to be changed to be larger when working with the physical Neato. The Bump sensor was also an example of this. It worked reliably on the physical Neato, but showed unexpected behavior in the simulator. Another takeaway was working on organizing code. There were some issues with organizing the ROS2 package and code structure correctly from the start. These issues were fixed, but cost time that could have been spent debugging.

## Individual Behaviors
### Behavior 1: Emergency Stop

Description:
The emergency stop is a safety behavior that takes precedence over every other behavior in the finite state machine. It allows the Neato to be stopped immediately, regardless of what it is currently doing. Its intent is to guarantee the robot never continues moving when it has been physically bumped, without requiring every individual behavior to implement its own safety logic.

Implementation:
	The node subscribes to two topics: Bump and estop. The process_bump method checks whether any of the four bump sensor fields (left_front, right_front, left_side, right_side) are triggered. The handle_estop method checks whether any incoming boolean message is true. Either condition sets an estop and immediately publishes a command velocity of zero. Separately, run_loop checks whether the estop is set as its very first line and returns immediately if true. No other state’s logic executes once the estop has been triggered. The estop condition is set as a threaded Event.

Key Design Decisions:
	The estop check is placed at the beginning of run_loop instead of being checked within each of the four behaviors. This guarantees that the estop always takes priority without needing to be duplicated across every state. The zero-velocity command is also published directly from the sensor callbacks, so the stop command goes out immediately.

Demo:
	https://youtube.com/shorts/1X_KKTO2R_c?feature=share

### Behavior 2: Person Following

Description:
	This behavior lets the Neato locate a nearby person using its LiDAR, and follow them while maintaining a roughly constant distance. Beyond basic tracking, the robot also smoothly scales its speed based on how far away the target is, and briefly continues turning toward a person's last known position if it loses track of them for a certain grace period, rather than stopping immediately. Its intent is to demonstrate reactive tracking behavior. The robot searches for something to follow, then continuously adjusts its heading and speed to keep the person in front of it at a target distance of 0.2 m.

Implementation:
	The node subscribes to the scan topic and publishes velocity commands to cmd_vel. This behavior spans two states. SEARCH is where the robot drives forwards at a low speed until process_scan reports something within a 0.6 m radius. FOLLOW is where the robot tracks the target for thirty seconds. The process_scan method loops over all 361 readings of the LIDAR each scan, discarding invalid readings and recording the closest valid reading’s distance and angle as self.obj_dist and self.obj_angle. The follow_person method reads these shared attributes and computes a proportional angular velocity based on how far off-center the target is, plus a velocity that scales proportionally with how far the person is. This is clamped between 0 and a maximum speed so the robot doesn’t stop abruptly or accelerate without bound if the person is too far away. Process_scan also records a last seen angle and last seen timestamp of the target. If the target is temporarily lost, the robot turns in that direction within a short grace period of 0.7 seconds. The Neato only stops after the grace period is over. All turning constants are hardcoded as instance attributes in __init__. 

Key Design Decisions:
	Proportional linear velocity was used because it produces a smoother following behavior. The last known angle grace period was added because with a single closest-point LIDAR reading, momentary sensor noise or a person briefly stepping just outside the 0.6m radius could otherwise cause the robot to stop and lose the person entirely. Allowing the robot to keep turning makes tracking noticeably more robust to brief signal dropouts without making the Neato chase something that is truly not there. The raw LIDAR data is also converted into a signed -180/+180 range before computing angular velocity to make the left/right distinctions much simpler. 

Demo:
	https://youtube.com/shorts/BxPyXvjqM9E

### Behavior 3: 180 degree turn

Description:
	This is a short transitional behavior that rotates the Neato in place by 180 degrees. Its intent is to reorient the robot after finishing the person-following behavior so that it faces the complete opposite of where the person walked off from. 

Implementation:
	This behavior does not subscribe to any new topics other than what the rest of the node already uses. However, it does publish to command velocity. The turn_left method commands a constant angular velocity and compares elapsed time against the time needed to complete π radians at that angular speed. Once that time is up, it stops the Neato and transitions to the DRAW state. 

Key Design Decisions:
	This was implemented as a timed, non-blocking behavior rather than using sleep to pause execution for the turn’s duration. A sleep call would not allow for the estop to work during the duration of the turn. Tracking elapsed time against a stored timestamp lets the estop callback still fire when needed. 

Demo:
	https://youtube.com/shorts/PTLDGjnhIY4 

### Behavior 4: Pentagon Shape

Description:
	The Neato is driven in a pentagon-shaped path, alternating between driving straight and turning five times. Its intent is to demonstrate open-loop trajectory execution by driving a predefined shape without taking in sensor feedback to know when to turn. 

Implementation:
	This only publishes Twist messages to command velocity. The path is defined once in __init__ as a list of tuples describing one side followed by a 72 degree turn. This list is then repeated five times to draw the whole pentagon. Draw_shape goes through this list one entry at a time using self.step using the same pattern as the turn behavior to know when to advance to the next step. Once all ten steps are complete, the robot stops and transitions to the DONE state. 

Key Design Decisions:
The pentagon is represented by a list of linear, angular, and duration steps rather than hardcoding five separate drive then turn calls. This made the shape easy to read and modify as the only changes would be with the degrees in the turn and the repeat count. This behavior is also timed to allow the estop to be called at any point in the drawing process. 

Demo:
	https://youtube.com/shorts/MimVg_-Lq70 

## Finite State Machine
### Overall Design
The intended behavior of the Neato running this finite state machine is a single and continuous sequence. The robot starts by driving slowly forward while scanning for a person nearby. It then locks onto this person and follows them for thirty seconds. After the thirty seconds, the Neato turns 180 degrees and traces out a pentagon before fully stopping. At any point, a physical bump or external estop signal can bring the Neato to a complete stop, overriding whichever state it is currently in. 

The States:

SEARCH - The robot drives forward at a fixed low speed with no turning, waiting until the LIDAR reports something within the detection radius

FOLLOW - The Neato actively tracks and follows the closest detected object within 0.6m while maintaining a 0.2m following distance, adjusting heading and speed proportionally for thirty seconds

TURN - The Neato executes a 180 degree turn in place

DRAW - The robot drives a pentagon-shaped path by alternating between driving straight and turning 72 degrees, five times

DONE - The robot stops and remains idle

The Transitions:

SEARCH → FOLLOW - Triggered when the LIDAR detects a valid object within the detection radius

FOLLOW →  TURN - Once thirty seconds have passed, regardless of whether the person is still being followed

TURN →  DRAW - After the time it takes to complete the 180 degree turn at the fixed turn speed

DRAW →  DONE - After all ten steps of the pentagon have been executed

Any State →  estop - Completely overrides any other behavior when either bumped or any other stop condition is returned true

<img width="1125" height="346" alt="image" src="https://github.com/user-attachments/assets/2322f3f3-fd7b-4a21-9aeb-da75f08865e8" />


### Implementation Details
The finite state machine is implemented as a single node. The current state is tracked by self.state, which is initialized to SEARCH. There is one timer that loops run_loop every 0.1 seconds. Run_loop dispatches to the correct state based on the current value of self.state. 

State transitions themselves are handled by a shared helper, set_state(self, new_state), which updates self.state, resets a timestamp that is used by any state that needs to track elapsed time within itself. It then resets self.step which is used by DRAW to track progress through its list of steps, and logs the transition via self.get_logger().info() for debugging. Every state method that needs to keep track of its time compares time.time() with self.state_start.

The emergency stop is implemented outside the state structure entirely. The run_loop checks if self.e_stop.is_set() is true as its first line. This Event is set by process_bump, which uses the Bump topic, and handle_estop, which uses the estop topic. This check happens once at the top of run_loop which lets it take priority over any behavior that is running. 
LIDAR sensor data is used in process_scan, which is used to find the distance of the object, the angle of the object, the time it was last seen, and the angle it was last seen at. 

### Demonstration
Full Physical FSM run-through: https://youtube.com/shorts/ZRA7LjG8dLs?feature=share

Overall, the finite state machine performs noticeably better in the simulator compared to the physical Neato. In the simulator, the TURN and DRAW states are as expected. The 180-degree turn and pentagon trajectory both execute cleanly and return the robot to its expected heading and position. On the physical robot, however, these two states are considerably less accurate, since they rely purely on timing rather than actual measured motion. Without odometry feedback, small discrepancies between the physical robot's real speed and our assumed constants accumulate over the course of the turn and each side of the pentagon, causing the real Neato's final heading and shape to drift noticeably from the intended result. This issue does not show up in the simulator where velocity commands translate to motion far more consistently. 

The person-following behavior is far choppier than preferred even with proportional control of both the linear and angular velocities. Rather than smooth tracking, the robot's motion tends to lurch slightly as it corrects its heading and distance. 

The main improvement to be made in the future is switching out the timing-based approach in TURN and DRAW with odometer feedback instead. Tracking the Neato’s actual heading and position rather than assuming a fixed velocity for a set time produces a fixed amount of movement. Another improvement would be to start recording rosbags earlier rather than solely at the end. Having the recorded sensor and command data would have made it easier to debug issues like the Bump sensor. 

## Learning Objectives and Final Takeaways

Pia - My learning objectives were to get comfortable with the basics of ROS2, like creating the files, building and sourcing, running the simulator, and thinking in terms of nodes, subscribers and publishers. I also wanted to learn how to connect and run code on a physical neato and to use rviz2 to visualize data for debugging. A key takeaway from this project is creating a good sketch and file layout/pseudocode for the behaviors first before jumping straight into coding a part of it. We jumped straight into coding the different behaviors and ended up having to reorganize our file system and merge files. I also learnt that the accuracy of the physical neato is not great, so the 180 degree turn or angles of the pentagon were not accurate in real life due to external factors like friction. In the future, I would like to use odom data to improve this. 

Avani - My learning objectives were to learn more about ROS2, understanding the difference between the Neato simulator and the real Neato, figuring out how to write a node that subscribes and publishes, and coding efficiently. I was able to get a lot more comfortable with ROS2 with the freedom of this project. We had some issues in the beginning of testing our code with the actual Neato. The radius for detecting an object was too small and that was not an issue in the simulator. I learned about trying to test earlier rather than later and not relying on the simulator too much. It was also good to learn more about how to write a node from scratch. 


## Downloading, Building, and Running our Code

### Download

Clone this repo into the `src` folder of a ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone <https://github.com/piaswarup/comprobo26_FSM_piavani>
```

### Build

From the root of the workspace:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Run

Make sure a Neato (real or simulated) is publishing `scan`, `bump`, and `estop`, then launch the FSM node:

```bash
ros2 run ros_behaviors_fsm finite_state_controller
```

The robot will search for the nearest object within its detection radius, follow it for a set amount of time, turn 180 degrees, then drive a pentagon before stopping. While following, the node also publishes a `visualization_msgs/Marker` on `/visualization_marker` showing the tracked person's location.

### Bagfiles

Recorded demo runs live in [`ros_behaviors_fsm/bags/`](ros_behaviors_fsm/bags/):

- `bump_estop_demo` — bump sensor triggering the e-stop
- `finite_state_controller_demo` — a full SEARCH → FOLLOW → TURN → DRAW → DONE run
- `pentagon_demo` — the DRAW state tracing out the pentagon
- `person_follower_demo` — the FOLLOW state tracking a person, including the visualization marker

To play a bag back, first disconnect from the (real or simulated) robot, then use the `--clock` flag so ROS uses the recorded timestamps rather than wall-clock time, and visualize the result in rviz:

```bash
ros2 bag play ros_behaviors_fsm/bags/<bag-file-name> --clock
```
