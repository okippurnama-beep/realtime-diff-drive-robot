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
- Simulated 360-sample 2D LiDAR publishing `/scan` at approximately 10 Hz
- Gazebo-to-ROS LaserScan bridge with `lidar_link` frame override
- RViz2 LaserScan visualization verified against a test obstacle
- Simulated IMU publishing `/imu` at approximately 100 Hz
- Dedicated Gazebo-to-ROS IMU bridge with `imu_link` frame override
- Static gravity, commanded yaw-rate, braking recovery, and `base_link -> imu_link` TF validation
- Wheel odometry and simulated IMU fused with `robot_localization`
- `/odometry/filtered` published by `ekf_filter_node` at approximately 50 Hz
- `odom -> base_footprint` TF now published by the EKF, with `diff_drive_controller` odom TF disabled to avoid duplicate TF publishers
- IMU covariance relay node normalizes Gazebo IMU covariance from `/imu/raw` to `/imu`

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

This launch file starts the custom Gazebo world, RViz2, `robot_state_publisher`, the `/clock`, `/scan`, and `/imu/raw` bridges, the IMU covariance relay node, `robot_localization`, and both ros2_control controllers.

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

## Verify Simulated LiDAR

```bash
ros2 topic info /scan
ros2 topic echo /scan --once --field header --qos-reliability best_effort
ros2 topic hz /scan --window 50
```

Expected results:

- `/scan` has one publisher
- The message frame is `lidar_link`
- The publishing rate is approximately 10 Hz

## Verify Simulated IMU

```bash
ros2 topic info /imu
ros2 topic echo /imu --once --field header
ros2 topic echo /imu --once --field linear_acceleration
ros2 topic echo /imu --once --field angular_velocity
ros2 topic hz /imu --window 200
ros2 run tf2_ros tf2_echo base_link imu_link
```

Expected baseline results:

- `/imu` has one publisher with type `sensor_msgs/msg/Imu`
- The message frame is `imu_link`
- The measured publishing rate is approximately 100 Hz
- Stationary linear acceleration is approximately `[0, 0, 9.8] m/s^2`
- Stationary angular velocity is approximately zero
- `base_link -> imu_link` translation is `[0, 0, 0.065]`

For a dynamic yaw-rate check, command an in-place rotation:

```bash
ros2 topic pub --rate 10 \
  /diff_drive_base_controller/cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.0}, angular: {z: 0.5}}}"
```

While the robot rotates, inspect the IMU from another terminal:

```bash
ros2 topic echo /imu --once --field angular_velocity
```

In the recorded validation run, the measured rate was
`100.001-100.010 Hz`, and a commanded yaw rate of `0.5 rad/s`
produced an IMU Z-axis reading of `0.50000015 rad/s`. After the
command publisher stopped, the angular velocity returned to
approximately zero. This is an intentionally ideal, noise-free
simulation baseline.

## Verify EKF Localization

```bash
ros2 topic info /imu/raw
ros2 topic info /imu
ros2 topic echo /imu --once --field orientation_covariance
ros2 topic echo /diagnostics --once --filter "any('ekf_filter_node' in s.name for s in m.status)" | grep -E 'level:|name:|message:'
ros2 topic hz /odometry/filtered --window 200
ros2 run tf2_ros tf2_echo odom base_footprint
```
Expected results:
- `/imu/raw` has one publisher and one subscriber
- `/imu` has one publisher and one subscriber
- `/imu` covariance fields use non-zero diagonal fallback values in simulation
- `ekf_filter_node` diagnostics report level 0
- `/odometry/filtered` publishes at approximately 50 Hz
- `odom -> base_footprint` is available from the EKF

In the recorded validation run, `/odometry/filtered` published at approximately 50 Hz. A commanded yaw rate of `0.5 rad/s` produced a filtered angular Z velocity of `0.50000532 rad/s`. After stopping, the filtered angular velocity returned to approximately zero.

## Known Jazzy Compatibility Workaround

The current simulation uses the tracked symbolic link:

```text
src/diffbot_controllers.yaml
  -> robot_description/config/controllers.yaml
```

This is a temporary workaround for controller parameter forwarding behavior in the current Jazzy `gz_ros2_control` integration.

## Roadmap

- Add SLAM Toolbox and Nav2
- Implement the STM32 motor-control firmware
- Implement encoder acquisition and PID control
- Add FreeRTOS tasks, watchdogs, and safety mechanisms
- Connect the MCU through micro-ROS
- Implement a real ros2_control hardware interface
- Perform fault-injection and quantitative tests
