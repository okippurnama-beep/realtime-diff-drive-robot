# M7.2-M7.4 Runtime Safety Design

## Purpose and Scope

This document freezes the implementation contract for three runtime modules:

- M7.2 creates the ROS 2 safety-supervisor adapter and removes every velocity
  bypass around the collision monitor and supervisor.
- M7.3 adds steady-clock watchdogs for command, scan, odometry, and Nav2
  lifecycle health, together with machine-readable diagnostics.
- M7.4 adds emergency-stop priority, latched faults, and a deliberately strict
  reset handshake.

M7.2-M7.4 remain host-side engineering safety features. They do not replace a
physical emergency stop, motor-driver protection, or the independent watchdog
that must later run on the STM32.

The M6 navigation result remains the no-fault reference. A runtime safety
change is not accepted if it introduces a second base-command publisher or
breaks the no-fault navigation baseline.

## Frozen End-to-End Command Path

```text
Nav2 controller and recovery behaviors
  -> /cmd_vel_nav                       geometry_msgs/msg/Twist
  -> velocity_smoother
  -> /cmd_vel_smoothed                  geometry_msgs/msg/Twist
  -> collision_monitor
  -> /cmd_vel_collision_checked         geometry_msgs/msg/Twist
  -> safety_supervisor
  -> /cmd_vel_safe                      geometry_msgs/msg/Twist
  -> twist_to_twist_stamped_node
  -> /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped
  -> diff_drive_controller
```

The graph contract is:

1. Only `collision_monitor` publishes `/cmd_vel_collision_checked`.
2. Only `safety_supervisor` publishes `/cmd_vel_safe`.
3. Only `twist_to_twist_stamped_node` publishes the controller command topic.
4. No Nav2, teleoperation, benchmark, or test node may publish directly to the
   controller command topic.
5. The controller's independent `cmd_vel_timeout: 0.5` remains enabled as the
   fallback when the supervisor or bridge process dies.

M7.2 changes the collision monitor output and bridge input in the same commit.
There must never be an intermediate committed launch configuration with a
missing final publisher or an active bypass.

## Shared Runtime Interfaces

| Direction | Name | Type | Purpose |
| --- | --- | --- | --- |
| Input | `/cmd_vel_collision_checked` | `geometry_msgs/msg/Twist` | Collision-checked command. |
| Input | `/scan` | `sensor_msgs/msg/LaserScan` | Safety freshness evidence. |
| Input | `/odometry/filtered` | `nav_msgs/msg/Odometry` | Motion-estimate freshness evidence. |
| Input | `/safety/estop` | `std_msgs/msg/Bool` | Manual or hardware-adapter emergency stop. |
| Input service | `/lifecycle_manager_localization/is_active` | `std_srvs/srv/Trigger` | Localization stack health. |
| Input service | `/lifecycle_manager_navigation/is_active` | `std_srvs/srv/Trigger` | Navigation stack health. |
| Output | `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | Only command allowed to reach the bridge. |
| Output | `/safety/status` | `diffbot_interfaces/msg/SafetyStatus` | Typed state, active faults, latches, and ages. |
| Output | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Standard ROS health reporting. |
| Service | `/safety/reset` | `std_srvs/srv/Trigger` | Explicit, guarded fault reset. |

The future MCU heartbeat input is reserved but disabled in the simulation
profile. It will be enabled when the virtual-MCU transport is introduced.

## QoS Contract

| Interface | QoS |
| --- | --- |
| Checked command input | Reliable, keep last 10 |
| Scan input | Sensor-data QoS, best effort |
| Filtered odometry input | Reliable, keep last 10 |
| Safe command output | Reliable, keep last 10 |
| Safety status | Reliable, transient local, keep last 1 |
| Emergency stop | Reliable, transient local, keep last 1 |
| Diagnostics | Reliable, keep last 10 |

Transient-local emergency stop means a newly started supervisor receives the
last latched stop value when the publisher uses the same durability. A topic is
still not a certified hardwired emergency stop; the physical robot must retain
an electrical motor-disable path.

## Parameter Contract

The simulation profile uses the following parameter names and initial values:

| Parameter | Initial value | Owner |
| --- | ---: | --- |
| `control_frequency_hz` | 50.0 | M7.2 |
| `status_frequency_hz` | 10.0 | M7.2 |
| `command_timeout_sec` | 0.30 | M7.3 |
| `scan_timeout_sec` | 0.50 | M7.3 |
| `odom_timeout_sec` | 0.50 | M7.3 |
| `nav2_poll_period_sec` | 0.25 | M7.3 |
| `nav2_health_timeout_sec` | 1.00 | M7.3 |
| `reset_health_hold_sec` | 0.50 | M7.4 |
| `max_forward_velocity` | 0.25 m/s | M7.2 |
| `max_reverse_velocity` | 0.10 m/s | M7.2 |
| `max_angular_velocity` | 1.00 rad/s | M7.2 |
| `require_scan` | true | M7.2 |
| `require_odom` | true | M7.2 |
| `require_nav2_active` | true | M7.3 |
| `require_mcu_heartbeat` | false | Future MCU stage |

Topic and service names are parameters with the shared runtime-interface names
as defaults. Timeout and frequency parameters are validated as finite and
positive during node construction. Velocity limits must also be finite and
positive; an invalid safety configuration prevents node startup.

## Clock and Concurrency Contract

Safety ages and deadlines use `std::chrono::steady_clock`, never ROS time.
Pausing Gazebo or losing `/clock` must therefore make inputs stale rather than
freezing their apparent age.

ROS time is used only for message headers and recorded correlation with the
simulation timeline.

The node runs a 50 Hz wall timer. Subscriptions only copy the latest value and
steady timestamp under a mutex. The timer creates an immutable
`SafetySnapshot`, releases the mutex, evaluates `SafetyPolicy`, and publishes
the decision. Service-response callbacks update health evidence without ever
blocking the 50 Hz safety loop.

The design remains correct under a multi-threaded executor even though the
first launch uses a single process and the normal ROS executor.

## M7.2: ROS Adapter and No-Bypass Command Chain

### Module purpose

M7.2 turns the tested C++ policy into a running ROS node and makes it the final
host-side command authority. It proves nominal pass-through and graph
ownership. It does not yet claim fault-response latency results.

### Planned files

```text
src/diffbot_safety/
  config/safety_params.yaml
  launch/safety_supervisor.launch.py
  scripts/verify_runtime_command_chain.py
  src/safety_supervisor_node.cpp
  test/test_safety_supervisor_node.cpp
  test/test_command_chain_contract.py
```

Existing files changed atomically:

- `diffbot_navigation/config/nav2_params.yaml`: collision monitor output becomes
  `/cmd_vel_collision_checked`.
- `diffbot_navigation/launch/navigation.launch.py`: starts the supervisor and
  configures the bridge input as `/cmd_vel_safe`.
- `robot_description/config/controllers.yaml`: aligns the controller's final
  velocity envelope with the Nav2 smoother and supervisor (`-0.10..0.25 m/s`,
  `-1.0..1.0 rad/s`) for defense in depth.
- `diffbot_safety/CMakeLists.txt` and `package.xml`: add only the ROS interfaces
  already installed in the Jazzy environment.

### Node cycle

Every 20 ms the node:

1. Calculates command, scan, odometry, lifecycle, and future-MCU ages from the
   steady clock.
2. Creates a complete `SafetySnapshot`.
3. Calls the pure C++ policy exactly once.
4. Publishes either the approved command or an explicit all-zero command.
5. Publishes `SafetyStatus`; diagnostics may run at a lower configured rate.

The node publishes zero continuously while inhibited or latched. It does not
rely on downstream timeout for an ordinary detected fault.

For M7.2, `require_nav2_active` remains false until the asynchronous lifecycle
watchdog is added in M7.3. Scan and odometry freshness remain required.

### M7.2 tests and exit criteria

- Isolated node starts inhibited and publishes zero.
- Fresh scan, odometry, and a fresh zero command allow transition to `READY`.
- A subsequent in-range command appears unchanged on `/cmd_vel_safe`.
- NaN, infinity, or an out-of-range command never appears on the safe output.
- Static configuration tests verify the exact topic chain.
- A configuration-contract test verifies that Nav2, the supervisor, and the
  base controller use the same or progressively tighter velocity envelope.
- A runtime graph test verifies the publisher-count contract after launch.
- One nominal simulated navigation goal succeeds after insertion of the
  supervisor.
- Existing M6 package tests remain green.

### M7.2 implementation result (2026-09-27)

M7.2 is implemented and runtime-validated. The supervisor reached `READY`,
the executable graph gate verified all four point-to-point links, and the
`straight_east=(2.8, 0.0)` Nav2 goal returned `SUCCEEDED` with `error_code: 0`.
The post-goal status remained `READY` with no active or latched faults. The
four affected packages reported `104 tests, 0 errors, 0 failures, 7 skipped`;
the skips are the Jazzy default for the known slow cppcheck version.

Runtime inspection also exposed a reproducible false `ODOM_STALE` latch under
heavy graph-discovery load with the draft `0.20 s` threshold. The EKF itself
measured about 50 Hz before and after the event. M7.2 therefore records and
uses the revised `0.50 s` threshold described below, and the same graph load
was repeated without a fault latch.

## M7.3: Steady-Clock Health Watchdogs and Diagnostics

### Module purpose

M7.3 detects loss of control authority or required perception. It distinguishes
which input became untrustworthy and exposes enough evidence to reproduce the
event instead of reporting only that the robot stopped.

### Freshness rules

| Evidence | Timeout | Fault rule |
| --- | ---: | --- |
| Nonzero command | 0.30 s | `COMMAND_STALE` after the last nonzero command becomes stale. |
| Zero command | No motion fault | Output remains zero; an idle system is not failed merely because Nav2 stopped publishing. |
| Laser scan | 0.50 s | Missing or stale data produces `SCAN_STALE`. |
| Filtered odometry | 0.50 s | Missing or stale data produces `ODOM_STALE`. |
| Combined Nav2 health | 1.00 s | Missing, false, or stale manager response produces `NAV2_INACTIVE`. |

Timeout values are configuration and initial acceptance targets, not measured
claims. The test report must store configured thresholds beside measured
latencies.

M7.2 runtime validation revised the odometry threshold from `0.20 s` to
`0.50 s`. A graph-discovery workload caused a transient host scheduling gap
above `0.20 s` even though `/odometry/filtered` measured about 50 Hz before
and after the event. Keeping the false trip as evidence and revising the
configuration is preferable to silently resetting a latched safety fault.

### Nav2 lifecycle watchdog

The supervisor creates asynchronous `std_srvs/srv/Trigger` clients for:

- `/lifecycle_manager_localization/is_active`
- `/lifecycle_manager_navigation/is_active`

Both services are part of the installed Jazzy lifecycle manager. The health
timer polls each manager every 0.25 s with at most one outstanding request per
manager. The 50 Hz safety timer never waits for a service.

Combined Nav2 health is true only when both managers most recently returned
`success: true` and both responses are younger than 1.00 s. A false response,
missing service, thrown future, or stale last response makes the combined
health false. Once M7.3 is integrated, `require_nav2_active` becomes true in the
navigation profile.

### Diagnostics contract

`/safety/status` is published at 10 Hz and immediately on a state transition.
It contains state, active and latched fault masks, inhibition state, and input
ages. A negative age means the input has never been observed.

`/diagnostics` publishes a status named `diffbot_safety/supervisor`:

- `OK`: `READY`, no active or latched faults.
- `WARN`: `STARTUP_INHIBIT` while required inputs are arriving.
- `ERROR`: `FAULT_LATCHED` or `ESTOP_LATCHED`.

Diagnostic key/value fields include state name, active fault names, latched
fault names, every input age, both lifecycle-manager results, output inhibited,
and configured timeout values.

### M7.3 latency targets

These targets refer to publication of the first zero command, not physical
braking distance:

| Injected loss | Command-level target |
| --- | ---: |
| Nonzero command stream | No later than 0.34 s after the last command |
| Filtered odometry | No later than 0.54 s after the last message |
| Laser scan | No later than 0.54 s after the last message |
| Nav2 manager health | No later than 1.04 s after last confirmed health |

The extra 40 ms permits two 50 Hz safety cycles. Physical speed decay is
recorded as a separate metric.

### M7.3 tests and exit criteria

- Launch tests use shortened test-only timeouts and synthetic publishers.
- Command, scan, and odometry streams are stopped independently.
- Fake lifecycle-manager services exercise true, false, unavailable, and stale
  response paths.
- Each fault produces the correct active bit, latches the reason, and publishes
  zero within its configured bound plus two safety cycles.
- Simultaneous failures preserve multiple bits instead of overwriting the first
  reason.
- Diagnostic state and age fields agree with `SafetyStatus`.

### M7.3 implementation result (2026-09-27)

M7.3 is implemented. The supervisor now polls both Nav2 lifecycle managers
asynchronously, requires both responses to remain active and fresh, and never
waits for a service from the 50 Hz command gate. Command, scan, and odometry
freshness continue to use the steady clock, so pausing simulation time cannot
hide a missing input.

The node publishes the standard `diffbot_safety/supervisor` diagnostic with
state, fault masks, inhibition state, input ages, lifecycle-manager evidence,
and the configured timeouts. The navigation profile now sets
`require_nav2_active: true`.

Seven ROS GTests use shortened test-only deadlines to cover healthy managers,
independent command/scan/odometry loss, an inactive manager, a disappearing
manager service, and simultaneous sensor faults. All seven scenarios publish
zero and preserve the expected fault bits. These are automated command-level
tests; production-threshold latency remains an acceptance target until the
separate system benchmark records it.

## M7.4: Emergency Stop, Fault Latching, and Guarded Reset

### Module purpose

M7.4 prevents automatic restart after a fault. It defines how an operator stops
the robot, how the reason remains observable, and the exact evidence required
before motion may be armed again.

### Emergency-stop behavior

An asserted `/safety/estop` immediately sets `MANUAL_ESTOP`, changes the state
to `ESTOP_LATCHED`, and publishes zero without waiting for the next ordinary
status period. The 50 Hz timer continues publishing zero afterward.

Emergency stop has priority over every other state. Releasing the topic does
not clear the latch and never resumes the previous command.

The command-level target from receiving an asserted stop message to publishing
zero is at most 40 ms. This will be measured, not assumed.

### Reset handshake

`/safety/reset` uses `std_srvs/srv/Trigger`. A reset succeeds only when all of
the following are true:

1. The policy is currently `FAULT_LATCHED` or `ESTOP_LATCHED`.
2. Emergency stop is released.
3. Scan, odometry, and required lifecycle health are fresh.
4. A finite, in-range, explicit zero command has been received recently.
5. No active fault bit remains.
6. All required conditions have remained healthy continuously for 0.50 s.

M7.4 will extend the pure policy snapshot with recovery-health duration and
will reject a reset when the zero command itself is stale. This prevents a
disconnected command source from being mistaken for a deliberate safe command.

Rejected resets return `success: false` and a deterministic reason, for
example `reset rejected: MANUAL_ESTOP active`, `fresh zero command required`,
or `recovery hold 0.50 s not satisfied`. An accepted reset clears all latches,
returns `READY`, and continues outputting zero until a later new nonzero command
arrives.

There is no automatic reset parameter.

### M7.4 tests and exit criteria

- Emergency stop from `READY` produces zero and `ESTOP_LATCHED`.
- Emergency stop during startup also latches.
- Reset while stop is asserted is rejected.
- Releasing stop without calling reset does not resume motion.
- Reset with stale sensors, inactive Nav2, nonzero command, stale zero command,
  or an incomplete 0.50 s recovery hold is rejected with the expected reason.
- A valid reset clears active latches but does not replay the pre-fault command.
- A transient-local stop published before supervisor startup is observed and
  prevents arming.
- Restarting or killing the supervisor cannot bypass the controller's 0.5 s
  command timeout.

## Test Architecture

Three levels remain separate:

1. Pure C++ GTest validates `SafetyPolicy` without ROS or sleeps.
2. ROS launch tests validate QoS, steady-time behavior, services, topics, and
   response latency using synthetic inputs.
3. Gazebo system tests validate the real command graph, one nominal navigation
   goal, and later controlled fault injection against the M6 baseline.

Fault-injection utilities are test-only nodes and are never launched by the
normal navigation entry point.

## Planned Implementation Order

1. Implement and test the M7.2 node in isolation.
2. Atomically change the collision-monitor output, supervisor input/output,
   and bridge input; verify the runtime graph and one nominal goal.
3. Implement M7.3 steady-clock watchdogs, lifecycle clients, status, and
   diagnostics; run synthetic stream-loss tests.
4. Strengthen the policy reset contract and implement M7.4 emergency-stop and
   reset service tests.
5. Only after M7.2-M7.4 pass, begin the separate fault-injection runner and
   repeated measured safety benchmark.

## Explicit Non-Goals

- No physical hardware purchase or motor test is part of M7.2-M7.4.
- No claim of functional-safety certification is made.
- Nav2 controller tuning is not changed unless a safety integration regression
  proves a configuration conflict.
- Teleoperation arbitration is not added. A future command multiplexer must be
  placed before the collision monitor and supervisor, never after them.
- MCU heartbeat enforcement stays disabled until a real or virtual MCU source
  exists.
