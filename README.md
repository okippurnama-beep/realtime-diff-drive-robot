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
- Typed `SafetyStatus` interface and independently tested C++ safety policy
- 50 Hz ROS 2 safety command gate inserted after Nav2 collision monitoring
- No-bypass velocity chain with an executable runtime graph validator
- Steady-clock command, LiDAR, odometry, and Nav2 lifecycle watchdogs
- Latched emergency stop with guarded, no-command-replay recovery
- Isolated four-scenario fault injection with durable CSV evidence and an
  independent offline acceptance gate
- Versioned host/MCU command-state protocol with session, sequence, watchdog,
  fault, and reboot semantics
- Deterministic fake MCU transport with controllable loss, delay, and reboot
- Jazzy ros2_control `SystemInterface` plugin with bounded zero-speed activation
  handshake and non-blocking wheel command/feedback conversion

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
colcon build --symlink-install --packages-select \
  diffbot_interfaces diffbot_hardware robot_description \
  diffbot_safety diffbot_navigation
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
This is an isolated low-level controller check for `sim.launch.py`; do not use
it while the navigation stack is active because navigation commands must pass
through the safety chain documented below.

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

## Verify the M7.2-M7.4 Runtime Safety Gate

The navigation launch now owns one explicit command path:

```text
/cmd_vel_nav
  -> /cmd_vel_smoothed
  -> /cmd_vel_collision_checked
  -> /cmd_vel_safe
  -> /diff_drive_base_controller/cmd_vel
```

`collision_monitor` owns `/cmd_vel_collision_checked`, `safety_supervisor`
owns `/cmd_vel_safe`, and the Twist-to-TwistStamped bridge is the only
publisher to the base controller command topic. The supervisor evaluates the
pure C++ safety policy at 50 Hz and publishes an explicit zero command while
startup is inhibited or a fault is latched.

Start the complete stack:

```bash
ros2 launch diffbot_navigation benchmark.launch.py
```

In a second sourced terminal, run the read-only graph gate:

```bash
ros2 run diffbot_safety verify_runtime_command_chain.py
```

Expected result: all four point-to-point links print `PASS`, followed by
`Runtime command-chain validation: PASS`. Inspect the typed safety state with:

```bash
ros2 topic echo /safety/status --once
```

For a healthy idle stack, expect `state: 1` (`READY`), `active_faults: 0`, and
`latched_faults: 0`. The simulation odometry timeout is `0.50 s`: the EKF
publishes at about 50 Hz, while the wider threshold avoids false latching from
short non-real-time Linux scheduling gaps. This is a configured acceptance
threshold, not a measured response-latency claim.

The recorded M7.2 acceptance run passed the live graph gate and then reached
`straight_east=(2.8, 0.0)` with `SUCCEEDED`, Nav2 `error_code: 0`, and no
active or latched safety faults. The four-package regression suite reported
`104 tests, 0 errors, 0 failures, 7 skipped`; the skipped items are Jazzy's
default handling of a known slow cppcheck version.

M7.2 deliberately introduced the no-bypass command gate before operator
E-stop/reset behavior was enabled in M7.4.

M7.3 adds steady-clock watchdogs for nonzero commands, laser scans, filtered
odometry, and both Nav2 lifecycle managers. The lifecycle checks use
asynchronous `/is_active` service requests so the 50 Hz command gate never
waits on a service. Both managers must report active within the configured
`1.0 s` health window before the supervisor can remain healthy.

Inspect the standard diagnostic record with:

```bash
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray \
  --filter "any(s.name == 'diffbot_safety/supervisor' for s in m.status)" \
  --once
```

A healthy stack reports `name: diffbot_safety/supervisor`, `message: READY`,
diagnostic level `OK`, both manager-active fields as `true`, and fresh scan and
odometry ages. The record also exposes fault masks, output inhibition, pending
lifecycle requests, service messages, input ages, and configured timeouts.

Seven synthetic ROS GTests cover healthy managers, independent command/scan/
odometry loss, an inactive manager, a disappearing manager service, and
simultaneous sensor loss. The recorded system acceptance kept all four command
links at `PASS`, reached `straight_east=(2.8, 0.0)` with `SUCCEEDED` and
`error_code: 0`, and remained `READY` with no active or latched faults. The
four-package regression suite reported `116 tests, 0 errors, 0 failures, 8
skipped`; the skipped items are Jazzy's default handling of a known slow
cppcheck version.

M7.4 adds a reliable, transient-local `/safety/estop` input and guarded
`/safety/reset` service. Assert emergency stop with:

```bash
ros2 topic pub --once --qos-reliability reliable \
  --qos-durability transient_local \
  /safety/estop std_msgs/msg/Bool "{data: true}"
```

Expect `ESTOP_LATCHED`, `MANUAL_ESTOP`, and a zero command. Publishing
`{data: false}` releases the physical/operator input but intentionally does not
clear the latch. A reset is accepted only after every required input is fresh,
Nav2 is active, and a new explicit zero command has remained healthy for
`0.50 s`:

```bash
ros2 service call /safety/reset std_srvs/srv/Trigger "{}"
```

Rejected calls return a deterministic explanation. A successful reset returns
`reset accepted`, enters `READY`, and still outputs zero until a later command
arrives; it never replays the command from before the fault. Six synthetic ROS
GTests cover READY/startup E-stop, retained E-stop delivery, release behavior,
rejection paths, the health-hold window, and no-command-replay recovery. The
command-level E-stop test enforces the designed `40 ms` test bound.

The recorded Gazebo acceptance run started from `READY`, passed the four-link
runtime graph gate, entered `ESTOP_LATCHED` with `MANUAL_ESTOP`, and published
zero. Reset was rejected both while E-stop remained asserted and after release
without a fresh zero. A test-only zero stream at the supervisor input then
satisfied the `0.50 s` recovery hold; reset returned `reset accepted`, state
returned to `READY`, output remained zero, and the graph gate passed again after
the temporary publisher was removed. The four-package regression suite
reported `129 tests, 0 errors, 0 failures, 9 skipped`.

This is a host-side engineering safety mechanism, not a hardware E-stop or a
functional-safety certification claim. The production fault-latency table
is measured separately below and remains a command-level simulation result.

## Verify the M7.5-M7.6 Fault-Injection Benchmark

M7.5 runs controlled faults only through a dedicated test profile. Normal
`benchmark.launch.py` behavior and the production velocity chain are unchanged.
The test relays `/scan` and `/odometry/filtered`, and proxies the two lifecycle
health services so one required source can be removed without killing an
unrelated process.

Check that the complete stack and all relays are ready without commanding
motion:

```bash
ros2 launch diffbot_navigation run_safety_benchmark.launch.py dry_run:=true
```

Run the formal four-scenario benchmark with three repetitions:

```bash
ros2 launch diffbot_navigation run_safety_benchmark.launch.py \
  repetitions:=3 \
  output_csv:=$PWD/benchmark_results/safety_benchmark_run.csv
```

Each trial requires a recent nonzero `/cmd_vel_safe` command before injecting
one of `estop`, `scan_timeout`, `odom_timeout`, or `nav2_timeout`. The runner
records monotonic injection, fault-status, and first-zero timestamps; cancels
the navigation goal; restores the evidence source; then requires a guarded
reset and clean `READY` state before continuing. Every CSV row is flushed and
`fsync`ed before the next trial.

Validate the evidence and generate reports offline:

```bash
ros2 run diffbot_safety summarize_safety_benchmark.py \
  benchmark_results/safety_benchmark_run.csv \
  --expected-repetitions 3 \
  --output-json benchmark_results/safety_benchmark_run.json \
  --output-markdown benchmark_results/safety_benchmark_run.md
```

The 2026-09-27 formal run passed all 12 trials with no validation warnings:

| Fault | Trials | Limit | Mean first-zero latency | P95 | Maximum |
| --- | ---: | ---: | ---: | ---: | ---: |
| Manual E-stop | 3/3 | 40 ms | 0.815 ms | 0.967 ms | 0.977 ms |
| LiDAR timeout | 3/3 | 540 ms | 466.520 ms | 497.954 ms | 500.000 ms |
| Odometry timeout | 3/3 | 540 ms | 500.131 ms | 500.242 ms | 500.251 ms |
| Nav2 health timeout | 3/3 | 1040 ms | 878.260 ms | 948.506 ms | 958.510 ms |

The raw evidence and generated reports are stored as
`benchmark_results/safety_benchmark_20260927.{csv,json,md}`. The final
four-package regression suite reported `138 tests, 0 errors, 0 failures, 9
skipped`; the skipped checks are Jazzy's known slow-version cppcheck handling.

These values measure the host-side ROS command path from the controlled
injection boundary to the first zero observed on `/cmd_vel_safe`. They do not
measure wheel deceleration, stopping distance, STM32/micro-ROS latency, or
functional-safety certification performance.

## Verify the M8.1-M8.4 Virtual MCU Boundary

M8 currently provides a versioned fixed-size MCU contract, a deterministic
in-process fake MCU, and the exported
`diffbot_hardware/DiffbotSystemHardware` ros2_control plugin. Verify the package
with:

```bash
colcon build --symlink-install --packages-select \
  diffbot_interfaces diffbot_hardware
source install/setup.bash
colcon test --packages-select diffbot_hardware
colcon test-result --test-result-base build/diffbot_hardware --verbose
```

The functional tests cover protocol validation, session/sequence rules, MCU
watchdog and encoder behavior, controlled transport loss/delay/reboot, pluginlib
discovery, and a real `ResourceManager` lifecycle/read/write cycle. Activation
requires a zero `DISARM` acknowledgement followed by a zero `ARMED`
acknowledgement before nonzero wheel targets can be sent. The realtime read and
write callbacks use cached interfaces and do not wait for recovery; stale
feedback, a changed boot ID, a session mismatch, an MCU fault, or an invalid
command returns `ERROR` for lifecycle handling.

M8.4 keeps the existing Gazebo path unchanged and adds a standalone no-bypass
profile:

```bash
source install/setup.bash
ros2 launch robot_description fake_mcu.launch.py headless:=true
```

In another terminal, confirm both controllers, the single-owner heartbeat
edge, and the M7 status:

```bash
source install/setup.bash
ros2 control list_controllers
ros2 topic info /mcu/state --verbose
ros2 topic echo /safety/status --once
```

Only validated, fresh, session-matching `ARMED` feedback is published by the
hardware plugin. M7 then rejects malformed, faulted, or duplicate state frames
before refreshing the heartbeat. This profile is an interface and failure-path
test; it is not a physics simulation, virtual-transport latency result, or
physical stopping claim.

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
- [x] Add the M7.1 safety contract and M7.2 no-bypass runtime command gate
- [x] Add M7.3 lifecycle/data watchdogs and standard diagnostics
- [x] Add M7.4 E-stop/controlled reset
- [x] Add controlled sensor/Nav2 fault injection and a repeated safety benchmark
- [x] Freeze the M8 virtual-MCU protocol, fault model, and `SystemInterface` boundary
- [x] Implement the deterministic fake MCU core and transport
- [x] Implement and lifecycle-test the ros2_control `SystemInterface`
- [x] Add the separate fake-hardware launch path and M7 MCU-heartbeat integration
- Implement the STM32 motor-control firmware
- Implement encoder acquisition and PID control
- Add FreeRTOS tasks, watchdogs, and safety mechanisms
- Connect the MCU through micro-ROS
- Replace the fake transport with the real STM32 hardware path
