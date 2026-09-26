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
- SLAM Toolbox mapping and Nav2 map saving validated
- Dedicated `diffbot_navigation` package for Nav2 bringup and configuration
- Automatic AMCL initialization for the fixed simulation spawn pose
- Robot-specific rectangular footprint, conservative velocity limits, and tighter goal tolerances
- Regulated Pure Pursuit control with pose-aware progress checking and filtered-odometry feedback
- Static global planning separated from live local obstacle and collision monitoring
- Full `/navigate_to_pose` control path validated with the Nav2 action returning `SUCCEEDED`
- Deterministic `8.2 x 6.2 m` structured navigation map and matching Gazebo world
- One-source benchmark layout with automatic clearance and reachability checks for five fixed goals
- One-command benchmark simulation, localization, and Nav2 startup
- Durable CSV logging plus offline Markdown/JSON validation reports
- Three-repetition navigation baseline with 15/15 measured goals and 15/15
  return-to-start actions completed successfully

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
colcon build --symlink-install --packages-select robot_description diffbot_navigation
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

## Verify SLAM Mapping

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
ros2 topic info /map
ros2 topic echo /map --once --field info
ros2 run tf2_ros tf2_echo map odom
ros2 topic hz /map --window 20
mkdir -p ~/robot_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/maps/diffbot_slam_test --ros-args -p save_map_timeout:=10.0
```

Expected results:

- `/map` has one publisher with type `nav_msgs/msg/OccupancyGrid`
- `map -> odom` TF is available
- `/map` publishes at approximately `0.2 Hz` with the default SLAM Toolbox settings
- The test map is saved as `maps/diffbot_slam_test.pgm` and `maps/diffbot_slam_test.yaml`

In the recorded validation run, SLAM Toolbox saved a `33 x 10` occupancy grid at `0.05 m/pix`.

## Verify the Structured Nav2 Baseline

```bash
ros2 launch diffbot_navigation benchmark.launch.py
```

The launch file loads the generated benchmark world, spawns the robot at
`(0.30, 0.00, 0.0)`, waits eight seconds for the simulator and controllers,
then starts AMCL and Nav2 with the matching occupancy grid.

In another sourced terminal, send the first fixed goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Expected results:

- AMCL initializes automatically at the fixed simulation start pose `(0.3, 0.0, 0.0)`.
- The localization and navigation lifecycle managers report `Managed nodes are active`.
- The map server loads a `164 x 124` map at `0.05 m/pix`.
- `twist_to_twist_stamped_node.py` is started by the navigation launch file.
- `/navigate_to_pose` accepts the goal and returns `SUCCEEDED` with `error_code: 0`.

The benchmark layout defines these fixed targets:

| Name | X (m) | Y (m) | Yaw (rad) | Scenario |
| --- | ---: | ---: | ---: | --- |
| `straight_east` | 2.80 | 0.00 | 0.0000 | Straight tracking |
| `east_detour` | 2.80 | -2.00 | -1.5708 | Wall detour |
| `south_west_corridor` | -3.00 | -2.40 | 3.1416 | Narrow corridor |
| `north_west_turn` | -2.50 | 2.00 | 1.5708 | Multi-turn route |
| `north_east_corridor` | 3.40 | 2.20 | 0.0000 | Wall and pillar avoidance |

The first recorded structured-world run reached `straight_east` with action
status `4` (`SUCCEEDED`) and stopped at approximately `(2.720, -0.002)` in the
`map` frame, or about `0.080 m` planar error. This was the pre-automation
functional check; the repeated benchmark results are recorded below.

The Gazebo world and occupancy map are generated from one reviewed layout file.
After editing the layout, regenerate the assets and verify that committed files
are current:

```bash
python3 src/diffbot_navigation/scripts/generate_benchmark_assets.py
python3 src/diffbot_navigation/scripts/generate_benchmark_assets.py --check
```

The `--check` command also inflates every obstacle by the configured `0.30 m`
validation clearance and confirms that all five goal cells remain connected to
the start. The same check runs under `colcon test`.

## Run the Automated Navigation Benchmark

For a complete run, including a return to the fixed start after every measured
goal, use:

```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch diffbot_navigation run_benchmark.launch.py repetitions:=1
```

The runner waits for `/navigate_to_pose`, executes all five goals in layout
order, writes each completed row immediately, and shuts down the launch after
the run. The one-command entry point runs Gazebo server-only and skips RViz so
that automated trials do not depend on a desktop or GPU window. Before sending
the first goal, it verifies all localization/navigation lifecycle nodes, both
ros2_control controllers, `/navigate_to_pose`, and `map -> base_footprint`.
By default, results are written to a timestamped CSV under
`benchmark_results/`. An explicit output path can be supplied with
`output_csv:=/absolute/path/results.csv`.

Each CSV row contains the target and measured start/finish poses, Nav2 action
status and error code, navigation and wall-clock durations, position and yaw
errors, maximum recovery count, timeout state, and return-to-start status.
The header and every completed row are flushed and synchronized to disk before
the next trial begins, so a later failure does not discard earlier evidence.
The runner stops if returning to the fixed start fails because later trials
would no longer share a comparable initial condition.

To validate configuration without moving the robot:

```bash
ros2 launch diffbot_navigation run_benchmark.launch.py dry_run:=true
```

This health check sends no motion goal and writes no benchmark CSV. At shutdown,
the launch file first requests `stop: true` through Gazebo's `/server_control`
service and then tears down the ROS graph, preventing old simulator servers and
controller managers from contaminating the next run.

To select individual goals without launching a second simulator, run the node
against an already active benchmark stack:

```bash
ros2 run diffbot_navigation nav_benchmark_runner.py --ros-args \
  -p dry_run:=true \
  -p "goal_names:=['straight_east','east_detour']" \
  -p repetitions:=2
```

The first runner smoke test executed only `straight_east`, with
`return_to_start:=false`. It recorded `SUCCEEDED`, Nav2 error code `0`,
`114.504 s` navigation time, `0.0755 m` final position error,
`0.00475 rad` yaw error, and zero recoveries. This validates the measurement
pipeline but is not a repeated benchmark result.

The recorded one-command dry run confirmed all 12 managed Nav2 lifecycle nodes,
both ros2_control controllers, the action server, and the required TF. Gazebo
returned `data: true` to the stop request, and a host-process check found no
remaining Gazebo server, GUI, or bridge process after launch exited.

## Validate and Summarize Benchmark Results

After a complete run, validate the raw CSV and write reviewable Markdown and
machine-readable JSON reports:

```bash
ros2 run diffbot_navigation summarize_nav_benchmark.py \
  benchmark_results/nav_benchmark_<UTC>.csv \
  --expected-repetitions 3 \
  --output-markdown benchmark_results/nav_benchmark_<UTC>_summary.md \
  --output-json benchmark_results/nav_benchmark_<UTC>_summary.json
```

The command exits with code `0` only when every input run is complete and
internally consistent. It checks the schema version, trial numbering, ordered
coverage of all five layout goals, target coordinates, derived position/yaw
errors, action result consistency, return-to-start status, and whether every
measured start is within `0.12 m` and `0.15 rad` of the fixed start pose.
Reports are replaced atomically after being synchronized to disk.

Timing and pose-error statistics use successful trials only; recovery counts
include all observed trials. Both the overall result and each goal report the
trial count, success count, success rate, mean, standard deviation, median,
95th percentile, minimum, and maximum where applicable. Failed trials are
classified by timeout, action status, Nav2 error code, or return failure.

Targeted smoke tests with `return_to_start:=false` can be inspected with
`--allow-disabled-return`, but they still fail formal validation if they do not
contain the full ordered goal set. The recorded one-goal smoke CSV is therefore
correctly reported as incomplete rather than accepted as a benchmark.

### Recorded navigation baselines

The post-tuning validation run in
`benchmark_results/nav_benchmark_20260926T140003Z.csv` passed every integrity
gate: all five goals and all five returns succeeded, no action timed out, and
all measured starts stayed within `0.12 m` and `0.15 rad` of the fixed start.
The generated reports are:

- `benchmark_results/nav_benchmark_20260926T140003Z_summary.md`
- `benchmark_results/nav_benchmark_20260926T140003Z_summary.json`

This is a one-repetition engineering baseline, not a repeatability claim. Its
measured navigation times ranged from `12.527 s` to `39.250 s`, and final
planar errors ranged from `0.0574 m` to `0.1366 m`.

The formal repeated baseline is
`benchmark_results/nav_benchmark_20260926T141653Z.csv`. It ran three complete
repetitions without restarting Gazebo or Nav2: all `15/15` measured goals and
all `15/15` return-to-start actions succeeded, with no timeouts or validation
warnings. The generated reports are:

- `benchmark_results/nav_benchmark_20260926T141653Z_summary.md`
- `benchmark_results/nav_benchmark_20260926T141653Z_summary.json`

Across the 15 measured goals, mean navigation time was `24.950 s` and the 95th
percentile was `38.089 s`; mean planar error was `0.0672 m` and the 95th
percentile was `0.0807 m`. The `north_west_turn` route exposed the largest
variance (`24.122--46.490 s`) and one trial required 14 recoveries. It still
passed, but this variability remains a visible optimization target rather than
being hidden behind the aggregate success rate.

The earlier failed formal runs and targeted regression artifacts remain in
`benchmark_results/` so the controller, progress-checker, odometry-feedback,
costmap-layering, and AMCL tuning decisions are reviewable rather than
presented as unexplained values.

## Known Jazzy Compatibility Workaround

The current simulation uses the tracked symbolic link:

```text
src/diffbot_controllers.yaml
  -> robot_description/config/controllers.yaml
```

This is a temporary workaround for controller parameter forwarding behavior in the current Jazzy `gz_ros2_control` integration.

## Roadmap

- [x] Build a larger structured navigation world and deterministic map
- [x] Add a repeatable multi-goal runner, durable CSV logging, and validation
- [x] Run three repeated trials and publish the raw CSV plus summary reports
- Add a safety supervisor and controlled sensor/communication fault injection
- Implement a fake MCU transport and a ros2_control `SystemInterface`
- Implement the STM32 motor-control firmware
- Implement encoder acquisition and PID control
- Add FreeRTOS tasks, watchdogs, and safety mechanisms
- Connect the MCU through micro-ROS
- Replace the fake transport with the real STM32 hardware path
