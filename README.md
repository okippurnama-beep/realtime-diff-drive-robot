# Real-Time Differential-Drive Robot Platform

A staged robotics and embedded-systems portfolio project based on ROS 2 Jazzy, Gazebo Harmonic, ros2_control, FreeRTOS, micro-ROS, and STM32.

## Current Milestone

The first simulation milestone is complete:

- Custom differential-drive robot described with URDF/Xacro
- Gazebo Harmonic physics simulation
- RViz2 robot visualization
- `joint_state_broadcaster`
- `diff_drive_controller`
- Forward, turning, and automatic braking tests
- Closed-loop odometry
- `odom -> base_footprint -> base_link` TF chain
- One-command Gazebo and RViz startup

## Development Environment

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
- RViz2
- ros2_control
- CMake and colcon
- Python 3 and C++

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_ws
colcon build --symlink-install --packages-select robot_description
source install/setup.bash
```

## Run the Simulation

```bash
ros2 launch robot_description sim.launch.py
```

This launch file starts Gazebo, RViz2, `robot_state_publisher`, the clock bridge, and both ros2_control controllers.

## Verify the Controllers

```bash
ros2 control list_controllers
```

Expected controllers:

```text
joint_state_broadcaster       active
diff_drive_base_controller    active
```

## Send a Test Command

```bash
ros2 topic pub --rate 10 --times 20 \
  /diff_drive_base_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

The robot moves briefly and then stops automatically when the command timeout expires.

## Inspect Odometry

```bash
ros2 topic echo --once \
  /diff_drive_base_controller/odom \
  --field pose.pose.position
```

## Inspect TF

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Known Jazzy Compatibility Workaround

The current simulation uses the tracked symbolic link:

```text
src/diffbot_controllers.yaml
  -> robot_description/config/controllers.yaml
```

This is a temporary workaround for controller parameter forwarding behavior in the current Jazzy `gz_ros2_control` integration.

## Roadmap

- Add simulated LiDAR and IMU sensors
- Add sensor visualization and validation
- Add `robot_localization`
- Add SLAM Toolbox and Nav2
- Implement the STM32 motor-control firmware
- Implement encoder acquisition and PID control
- Add FreeRTOS tasks, watchdogs, and safety mechanisms
- Connect the MCU through micro-ROS
- Implement a real ros2_control hardware interface
- Perform fault-injection and quantitative tests
