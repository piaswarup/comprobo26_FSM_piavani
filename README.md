# comprobo26_FSM_piavani
Person following hopefully

## Downloading, Building, and Running

### Download

Clone this repo into the `src` folder of a ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone <this-repo-url>
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

The robot will search for the nearest object within its detection radius, follow it for a set amount of time, turn 180 degrees, then drive a pentagon before stopping.

### Bagfiles

No bagfiles are currently included in this repo, but any recorded runs should be pushed to a `bags/` subdirectory, named so it's clear which part of the assignment the run corresponds to (e.g. `bags/follow-turn-draw`).

To record a run, avoid the high-rate `/camera/image_raw` and `/gazebo/` topics (they bloat the file size fast) by recording only the relevant topics:

```bash
ros2 bag record /accel /bump /odom /cmd_vel /scan /stable_scan /projected_stable_scan /tf /tf_static -o bags/<bag-file-name>
```

To play a bag back, first disconnect from the (real or simulated) robot, then use the `--clock` flag so ROS uses the recorded timestamps rather than wall-clock time, and visualize the result in rviz:

```bash
ros2 bag play bags/<bag-file-name> --clock
```

Note: point the path at the bag directory itself, not the `metadata.yaml` or `.db3` file inside it. If a bag directory is much larger than ~50MB, something was likely captured that shouldn't have been (e.g. uncompressed images).
