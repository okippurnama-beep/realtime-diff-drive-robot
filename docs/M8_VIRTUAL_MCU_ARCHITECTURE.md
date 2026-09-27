# M8 Virtual MCU and ros2_control Hardware Architecture

## Purpose and Scope

M8 inserts the same host/MCU boundary that the physical robot will use before
STM32 hardware exists. The immediate goal is not a more realistic Gazebo
model. It is to prove that Nav2 and `diff_drive_controller` can reach wheel
feedback through a transport-independent `ros2_control` `SystemInterface`, and
that communication failures produce deterministic safe behavior.

M8.1 freezes the protocol, transport contract, failure model, and hardware
lifecycle boundary. M8.1 does not yet implement a plugin, fake MCU process,
micro-ROS transport, or measured latency benchmark.

## Module Plan

| Module | Deliverable | Acceptance evidence |
| --- | --- | --- |
| M8.1 | Fixed-size messages, pure C++ protocol/transport contract, architecture document | Interface generation, contract unit tests, package regression |
| M8.2 | Deterministic fake MCU core and in-process transport | Nominal motion, watchdog, reboot, sequence, drop/delay tests |
| M8.3 | `DiffbotSystemHardware` plugin | ros2_control lifecycle and joint interface tests |
| M8.4 | Fake-hardware launch path and M7 MCU-heartbeat integration | No-bypass graph, controller activation, safe fault propagation |
| M8.5 | Controlled transport fault runner and offline report | Repeated raw evidence with predeclared limits |
| M8.6 | Freeze the micro-ROS adapter boundary | Fake transport can be replaced without changing controllers or safety policy |

## Layering

```text
diff_drive_controller
  -> wheel velocity command interfaces
DiffbotSystemHardware : hardware_interface::SystemInterface
  -> HostCommandFrame / McuStateFrame
McuTransport
  -> M8.2 deterministic fake transport
  -> future micro-ROS transport
MCU control and safety state machine
  -> encoder counts and measured wheel velocity
```

The controller, hardware plugin, protocol, and transport are separate layers.
The hardware plugin must not know whether the other endpoint is the fake MCU
or an STM32. The transport must not implement robot safety policy or wheel
kinematics.

## Fixed-Size Protocol v1

The ROS messages are the versioned transport mapping. The pure C++ structs in
`diffbot_hardware/mcu_protocol.hpp` are the control-loop representation. Field
names, units, and integer widths must remain aligned.

### Host to MCU: `McuCommand`

| Field | Unit | Rule |
| --- | --- | --- |
| `protocol_version` | none | Must equal 1. |
| `host_session_id` | none | Nonzero and regenerated for each hardware activation. |
| `command_sequence` | none | Monotonically increases modulo 2^32 inside a session. |
| `valid_for_ms` | ms | MCU starts a local deadline when the frame is accepted. |
| `mode` | enum | `DISARM`, `ARMED`, or `ESTOP`. |
| `left_target_velocity_mrad_s` | mrad/s | Fixed-point wheel target. |
| `right_target_velocity_mrad_s` | mrad/s | Fixed-point wheel target. |

`DISARM` and `ESTOP` frames must contain zero wheel targets. An invalid version,
session, mode, deadline, sequence, or wheel target is rejected and cannot
refresh the MCU watchdog.

### MCU to Host: `McuState`

| Field | Unit | Rule |
| --- | --- | --- |
| `protocol_version` | none | Must equal 1. |
| `mcu_boot_id` | none | Nonzero and changes on every MCU boot. |
| `state_sequence` | none | Monotonically increases modulo 2^32 inside one boot. |
| `accepted_host_session_id` | none | Echoes the session currently owned by the MCU. |
| `last_accepted_command_sequence` | none | Echoes the last command that refreshed the watchdog. |
| `mcu_uptime_ms` | ms | MCU-local monotonic uptime; never compared directly with host time. |
| `mode` | enum | `BOOT`, `DISARMED`, `ARMED`, `FAULT`, or `ESTOP`. |
| `active_faults` | bit mask | Stable protocol-level MCU fault evidence. |
| `left/right_encoder_count` | count | Raw signed cumulative quadrature counts. |
| `left/right_velocity_mrad_s` | mrad/s | MCU-estimated wheel velocity. |
| `last_command_age_ms` | ms | MCU-local age of the last accepted command. |
| `control_loop_overrun_count` | count | Monotonic real-time loop overrun evidence. |

The message intentionally contains no ROS timestamp, string, or variable-size
array. The host stores a `steady_clock` receive timestamp beside each state
frame. The MCU evaluates command expiry using its own monotonic timer. This
avoids pretending the two clocks are synchronized.

## Initial Timing Budget

| Item | Initial value |
| --- | ---: |
| ros2_control update and host command rate | 100 Hz |
| Virtual/physical MCU control loop | 1000 Hz |
| MCU state publication | 100 Hz |
| Command validity and MCU watchdog | 100 ms |
| Host state freshness timeout | 200 ms |
| Maximum absolute commanded wheel velocity | 10,000 mrad/s |

These are configuration defaults and acceptance thresholds selected before
implementation. They are not benchmark results. M8.5 must store configured
limits beside measured evidence. Real STM32 values must be measured again.

## Session, Sequence, and Reboot Rules

1. A new hardware activation creates a nonzero `host_session_id`.
2. The MCU adopts a new session only from a valid zero `DISARM` command.
3. The host then sends a zero `ARMED` command and waits for the state echo
   before nonzero commands are allowed.
4. Duplicate or older commands do not refresh the watchdog. Sequence ordering
   uses modulo-2^32 half-range comparison so wraparound remains valid.
5. A changed `mcu_boot_id` invalidates the active session and wheel-state
   continuity. The host commands zero and repeats the bounded activation
   handshake; it never silently treats the reboot as ordinary packet loss.
6. A state frame with an unexpected session, version, boot ID, sequence, or
   unknown mode is not copied into ros2_control state interfaces.

## MCU Fault Contract

Protocol v1 reserves stable bits for command timeout, protocol mismatch,
command sequence error, invalid command, left/right encoder faults,
left/right driver faults, undervoltage, overcurrent, control-loop overrun,
MCU watchdog reset, and the physical E-stop input.

The MCU independently drives its motor outputs to zero when:

- the command deadline expires;
- an `ESTOP` command or physical E-stop is active;
- a frame is invalid or belongs to an unaccepted session;
- a configured motor, encoder, power, or real-time fault requires shutdown.

The host cannot substitute for this watchdog. Loss of host-to-MCU transport
must still stop the motor outputs even if the ROS process has crashed.

## McuTransport Contract

`McuTransport` is pure C++ and does not expose ROS messages to the realtime
read/write loop. After activation:

- `send_command()` and `receive_latest()` are bounded, non-blocking, and
  `noexcept`;
- they do not allocate memory, wait for discovery, sleep, or log per cycle;
- `receive_latest()` returns the newest complete frame plus the host steady
  receive time;
- `kNoData` is distinct from disconnect, protocol error, and I/O error;
- shutdown is idempotent and a transport object has one owner.

M8.2 implements the fake transport behind this interface. The later micro-ROS
adapter must satisfy the same behavior without changing the hardware plugin.

## SystemInterface Lifecycle Boundary

The M8.3 plugin will implement the installed Jazzy `SystemInterface` API:

- `on_init`: validate exactly two wheel joints, velocity command interfaces,
  position/velocity state interfaces, encoder scale, limits, and timeouts.
- `on_configure`: configure the transport but leave outputs disarmed.
- `on_activate`: execute a bounded zero/disarm/session/zero-arm handshake.
- `read`: consume the latest valid state without blocking, convert encoder
  counts to radians, update joint state, and reject stale or inconsistent
  feedback.
- `write`: validate finite wheel commands, convert rad/s to integer mrad/s,
  assign sequence/deadline/session fields, and perform one non-blocking send.
- `on_deactivate` and `on_shutdown`: best-effort zero/disarm before closing the
  transport; MCU watchdog remains the independent fallback.

Repeated I/O or freshness failures return `hardware_interface::return_type::ERROR`
instead of publishing invented wheel feedback. Lifecycle callbacks may wait
for their documented bounded handshake; `read()` and `write()` may not.

## M7 Integration Boundary

M8.4 will publish a host-side MCU heartbeat derived only from valid, fresh,
session-matching state frames. M7 will then enable `require_mcu_heartbeat`.
Receiving any message is insufficient: malformed, stale, wrong-session, or
rebooted state cannot keep the safety supervisor healthy.

The existing host velocity chain remains unchanged. Transport fault injection
is placed below `diff_drive_controller`, while M7 command and sensor fault
injection remains above it. This separation lets later reports distinguish
host command-path latency from MCU/transport behavior.

## M8.1 Exit Criteria

- Both fixed-size messages generate successfully.
- Pure C++ validation covers versions, modes, zero-command constraints,
  limits, fault masks, normal sequence progression, and wraparound.
- `diffbot_hardware` builds without depending on Gazebo or micro-ROS.
- Existing M6/M7 interfaces and tests remain unchanged and pass.
- No measured performance or physical-stop claim is added.

## M8.1 Implementation Result (2026-09-27)

M8.1 is implemented. `McuCommand.msg` and `McuState.msg` generate correctly,
and `ros2 interface show` confirms the intended fixed-size fields, units,
mode constants, and fault bits. The new `diffbot_hardware` package contains a
ROS-independent protocol validator and non-blocking `McuTransport` interface;
it does not yet contain a fake MCU or ros2_control plugin.

Eight GTest cases cover the ROS-message/C++ constant mapping, valid drive
frames, invalid versions and sessions, zero-only safe modes, command and
feedback limits, known fault masks, MCU boot metadata, ordinary sequence
ordering, and uint32 wraparound. The five-package M6/M7/M8 regression completed
with `167 tests, 0 errors, 0 failures, 12 skipped`. All 12 skips are the Jazzy
default exclusion of the installed cppcheck version with known performance
issues, not failed functional tests.

The timing values in this document remain predeclared design budgets. M8.1 did
not run a transport-latency or physical-stopping benchmark.

## M8.2 Implementation Result (2026-09-27)

M8.2 is implemented as a pure C++ deterministic fake MCU plus an in-process
`McuTransport`. The fake MCU owns the session and mode state machine, validates
every command, integrates ideal encoder counts, latches sequence and command
timeout faults, stops at the exact command deadline, and changes `mcu_boot_id`
while invalidating the host session on reboot.

The transport uses a fixed-capacity command queue. Its control-loop methods do
not sleep or wait for ROS discovery. A manually injected steady clock makes
command delay, command loss, state loss, watchdog expiry, and reboot tests
repeatable without wall-clock `sleep()` calls. M8.2 deliberately remains below
ROS topics and above no physical plant; the Gazebo navigation path is unchanged.

Eleven new GTest cases cover zero/disarm session ownership, wheel feedback,
exact watchdog stopping, duplicate sequences, delayed delivery, command and
state loss, and reboot observation. Together with the eight M8.1 protocol tests,
the hardware package now has 19 functional GTest cases. The final five-package
regression reported `204 tests, 0 errors, 0 failures, 18 skipped`; the skips are
the installed Jazzy cppcheck performance exclusion for the M7 and M8 C++ files.

One initial five-package run saw `ament_xmllint` receive an empty document from
the ROS package schema URL. The unchanged safety package's targeted xmllint and
complete package suite both passed immediately afterward, and the final CTest
record contains no failure. This was retained as an external validator-fetch
event, not misclassified as an M8 code defect.

No transport latency or physical motion result is claimed in M8.2. The next
module is M8.3, which connects this transport to the installed Jazzy
`hardware_interface::SystemInterface` lifecycle and wheel interfaces.
