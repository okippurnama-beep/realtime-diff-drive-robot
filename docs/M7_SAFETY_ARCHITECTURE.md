# M7 Safety Supervision and Controlled Fault Injection

## Status

M7.1 defines the safety contract and implements the ROS-independent policy
core. It does not change the running navigation command path. Runtime topic
wiring starts in M7.2 after the policy unit tests pass.

The detailed runtime contracts for M7.2, M7.3, and M7.4 are recorded in
[`M7_RUNTIME_SAFETY_DESIGN.md`](M7_RUNTIME_SAFETY_DESIGN.md).

This is an engineering safety layer for a research and portfolio robot. It is
not a certified functional-safety system and does not replace a physical
emergency stop, motor-driver protections, or the future STM32 watchdog.

## Problem Statement

The M6 navigation baseline can complete repeatable goals, but successful
navigation does not prove fail-safe behavior. The current bridge subscribes to
`/cmd_vel_smoothed`, while Nav2's collision monitor publishes its checked
command on `/cmd_vel`. That bridge therefore bypasses the collision monitor.

M7 must establish one command path with no alternate publisher able to reach
the base controller:

```text
Nav2 controller and behaviors
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_smoothed
  -> collision_monitor
  -> /cmd_vel_collision_checked
  -> safety_supervisor
  -> /cmd_vel_safe
  -> Twist-to-TwistStamped bridge
  -> /diff_drive_base_controller/cmd_vel
  -> diff_drive_controller timeout
  -> simulated or physical base
```

M7.2 will apply this change atomically. M7.1 deliberately leaves the M6
runtime graph untouched.

## Safety Responsibilities

- `collision_monitor` handles geometric collision risk from live obstacle
  data.
- `safety_supervisor` handles command validity, input freshness, system health,
  emergency stop, fault latching, and controlled reset.
- `diff_drive_controller` retains its independent command timeout if the
  supervisor or bridge dies.
- The future STM32 firmware must independently enforce communication timeout,
  watchdog, motor disable, and hardware emergency-stop behavior.

## State Machine

The policy has four states:

- `STARTUP_INHIBIT`: output is forced to zero until every required health input
  has been observed and is fresh.
- `READY`: a fresh, finite, in-range command may pass through.
- `FAULT_LATCHED`: output is forced to zero after a non-emergency fault. Input
  recovery alone does not resume motion.
- `ESTOP_LATCHED`: output is forced to zero after emergency stop. This state has
  priority over all other states.

A startup transition to `READY` requires a zero command (or no command yet) in
addition to healthy inputs. A reset is accepted only when the policy is
latched, all active faults have cleared, and the most recent command is zero.
Starting or resetting directly into a nonzero command is forbidden.

Startup freshness failures inhibit motion but are not latched. Invalid or
out-of-range commands are latched even during startup because they indicate an
untrusted upstream source.

## Fault Bit Contract

`diffbot_interfaces/msg/SafetyStatus` exposes active and latched fault masks.
Bit values are stable API contracts for tests and recorded evidence.

| Bit | Fault | Meaning |
| ---: | --- | --- |
| 0 | `MANUAL_ESTOP` | Emergency stop is asserted. |
| 1 | `COMMAND_STALE` | A nonzero command exceeded its freshness timeout. |
| 2 | `SCAN_STALE` | Required laser scan is missing or stale. |
| 3 | `ODOM_STALE` | Required filtered odometry is missing or stale. |
| 4 | `NAV2_INACTIVE` | Required Nav2 health is not active. |
| 5 | `INVALID_COMMAND` | Linear or angular command is NaN or infinite. |
| 6 | `COMMAND_LIMIT_VIOLATION` | Command exceeds the hard safety envelope. |
| 7 | `MCU_HEARTBEAT_LOST` | Required future MCU heartbeat is missing or stale. |

Multiple faults may be active simultaneously. `active_faults` represents the
current inputs; `latched_faults` preserves faults until a valid reset.

## Initial Policy Parameters

These values are initial acceptance targets, not measured results:

| Parameter | Initial value |
| --- | ---: |
| Command timeout while nonzero | 0.30 s |
| Laser scan timeout | 0.50 s |
| Filtered odometry timeout | 0.20 s |
| Future MCU heartbeat timeout | 0.20 s |
| Maximum forward velocity | 0.25 m/s |
| Maximum reverse velocity magnitude | 0.10 m/s |
| Maximum angular velocity magnitude | 1.00 rad/s |
| Zero-command epsilon | 0.0001 |

A stale zero command is safe and does not create `COMMAND_STALE`. A stale
nonzero command is a fault. The ROS adapter will calculate ages from a steady,
monotonic clock so pausing simulation time cannot hide stale inputs.

## M7.1 Package Boundaries

`diffbot_interfaces` owns the stable ROS message contract.

`diffbot_safety` currently contains a pure C++17 `SafetyPolicy`. It has no ROS
node and no ROS clock dependency. This separation allows deterministic unit
tests and makes the policy reusable by a later virtual-MCU or hardware adapter.

The future ROS adapter will translate subscribed messages into a
`SafetySnapshot`, call `SafetyPolicy::evaluate()`, and publish the resulting
command and `SafetyStatus`.

## M7.1 Unit-Test Contract

The policy tests must prove:

1. Startup remains inhibited until required inputs are fresh.
2. Startup cannot arm while a nonzero command is already present.
3. Commands pass only in `READY`.
4. Sensor faults latch after the system has reached `READY`.
5. Emergency stop has highest priority and cannot reset while asserted.
6. A stale nonzero command faults while a stale zero command remains safe.
7. Reset is rejected until health recovers and the command is zero.
8. NaN, infinity, and out-of-range commands cannot reach the output.
9. The future MCU heartbeat can be enabled without changing the policy API.
10. Invalid configuration is rejected during construction.
11. State and combined-fault names are deterministic for diagnostics.

## M7.2 Entry Criteria

M7.2 may begin only after both new packages build, every M7.1 unit and lint
test passes, generated `SafetyStatus` interfaces can be inspected with
`ros2 interface show`, and the existing M6 runtime files remain unchanged.

M7.2 will then add the ROS adapter and switch the command chain in one tested
change so there is no intermediate launch configuration without a valid final
velocity publisher.
