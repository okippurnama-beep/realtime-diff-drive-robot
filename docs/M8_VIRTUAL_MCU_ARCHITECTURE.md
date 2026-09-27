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
   continuity. The realtime `read()` returns `ERROR`; lifecycle recovery must
   then deactivate and execute a new bounded activation handshake. A reboot is
   never silently treated as ordinary packet loss inside the control loop.
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

No transport latency or physical motion result is claimed in M8.2.

## M8.3 Implementation Result (2026-09-27)

M8.3 implements and exports
`diffbot_hardware/DiffbotSystemHardware` as a Jazzy
`hardware_interface::SystemInterface` plugin. It validates an exact two-wheel
URDF contract, requires a velocity command plus position/velocity state for
each wheel, and requires an explicit encoder-count scale. Command validity,
host feedback timeout, and bounded activation timeout are configurable
hardware parameters with checked defaults.

`on_configure()` caches the six ros2_control interface handles and prepares a
new nonzero host session. `on_activate()` will not expose motion until the MCU
has echoed both a zero `DISARM` frame and a later zero `ARMED` frame from that
session, including the corresponding command sequence. Deactivation, shutdown,
error handling, and object destruction all make a best-effort zero `DISARM`
before closing the transport; the MCU watchdog remains the independent stop.

The 100 Hz `read()` and `write()` paths use cached handles with single-attempt,
non-blocking access. `write()` rejects NaN, infinity, and wheel targets beyond
10 rad/s before converting rad/s to integer mrad/s. `read()` validates protocol,
boot ID, session, state sequence, MCU mode, faults, receive timestamp, and the
200 ms default freshness budget before converting cumulative encoder counts to
radians and measured mrad/s to rad/s. A changed boot ID or stale feedback
returns `hardware_interface::return_type::ERROR`; the realtime loop never
blocks to repair a session.

Nine new GTests cover invalid wheel contracts, pluginlib discovery, a real
`ResourceManager` configure/activate/read/write cycle, zero handshake ordering,
command and feedback conversions, non-finite/over-limit rejection, stale state,
MCU reboot detection, and zero/disarm deactivation. Together with M8.1 and
M8.2, `diffbot_hardware` now has 28 functional GTest cases. M8.3 does not yet
replace the Gazebo hardware block or publish the M7 MCU heartbeat; that runtime
integration belongs to M8.4.

The final five-package M6/M7/M8 build and regression completed with
`227 tests, 0 errors, 0 failures, 21 skipped`. All 21 skips are the installed
Jazzy cppcheck slow-version exclusions for the M7/M8 C++ packages, not failed
functional tests. No transport-latency or physical-motion measurement was
added in M8.3.

## M8.4 Implementation Result (2026-09-27)

M8.4 adds a standalone `fake_mcu.launch.py` profile. The Xacro now emits
exactly one hardware owner: the original profile contains only
`GazeboSimSystem` and the Gazebo control plugin, while `use_fake_mcu:=true`
contains only `DiffbotMcuSystem`. This prevents a Gazebo joint path from
bypassing the virtual MCU boundary.

The hardware plugin publishes `/mcu/state` with a realtime publisher after a
frame passes protocol and receive-time validation. Healthy frames must also
pass boot ID, session, state-sequence, `ARMED`, and zero-fault checks before
they update ros2_control joint feedback. A protocol-valid frame that reports a
new boot, wrong session, non-`ARMED` mode, or MCU fault is exposed only as
negative safety evidence and still makes `read()` return `ERROR`; it never
updates joint feedback. The M7 supervisor immediately invalidates its MCU
heartbeat on that negative evidence. Malformed and duplicate frames cannot
refresh the heartbeat and age out under the 200 ms watchdog.

The standalone runtime was exercised with both controllers active. The ROS
graph had one `/mcu/state` publisher (the hardware component), one subscriber
(the safety supervisor), one `/cmd_vel_safe` publisher, and one command-adapter
subscriber. A 1 s, 0.12 m/s command produced equal wheel feedback positions of
2.01565 rad before a zero command. The deliberately slow CLI handoff also
triggered the existing command-stale latch; sustained zero input followed by
`/safety/reset` returned `reset accepted` and `READY`, showing that the M7 gate
remains authoritative in the M8 path.

New node-level tests prove that faulted MCU frames and repeated state sequence
numbers cannot refresh the heartbeat. The final targeted results were
`81 tests, 0 errors, 0 failures, 12 skipped` for `diffbot_hardware` and
`87 tests, 0 errors, 0 failures, 10 skipped` for `diffbot_safety`; skips remain
the installed Jazzy slow-cppcheck exclusions.

## M8.5 Implementation Result (2026-09-27)

M8.5 adds a typed `/mcu/fault_control` service on the fake transport and a
fresh-stack benchmark runner for command loss, state loss, 150 ms command
delay, and MCU reboot. Each trial first proves that both controllers are
active, M7 is `READY`, and a nonzero command is visible on `/cmd_vel_safe`.
It then records monotonic injection, first fault-status, and first-zero output
timestamps in an append-only CSV. An independent offline summarizer rejects
missing scenarios, duplicate trial indices, changed design parameters,
non-finite values, incomplete repetitions, or any result beyond the
predeclared 260 ms host-side limit.

The first complete run exposed that protocol-valid MCU fault frames were being
discarded at the hardware boundary. The physical/virtual MCU had already
stopped at its 100 ms watchdog, but M7 could see only a later 200 ms heartbeat
timeout. The corrected boundary now forwards an unhealthy frame as negative
safety evidence while still refusing to copy it into ros2_control feedback.
This reduced the repeated command-loss and delay paths from about 300 ms to
about 115 ms without weakening either watchdog or the acceptance limit.

The accepted run stored in
`benchmark_results/mcu_transport_benchmark_20260927T131809Z.{csv,json,md}`
passed all 12 trials:

| Transport fault | Trials | Mean first-zero (ms) | P95 (ms) | Maximum (ms) |
| --- | ---: | ---: | ---: | ---: |
| Command loss | 3/3 | 114.864 | 117.747 | 117.939 |
| State loss | 3/3 | 206.523 | 217.905 | 219.912 |
| 150 ms command delay | 3/3 | 115.292 | 119.250 | 119.259 |
| MCU reboot | 3/3 | 4.930 | 8.495 | 8.962 |

The measurement boundary is the host fault-service call to the first zero on
`/cmd_vel_safe`. These values do not measure physical wheel deceleration,
stopping distance, serial/micro-ROS latency, or a functional-safety guarantee.
