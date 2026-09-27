# M7.5-M7.6 Fault-Injection and Safety-Benchmark Design

## Purpose

M7.5 makes safety faults repeatable instead of relying on manually killing ROS
processes. M7.6 turns each injected event into durable, reviewable measurements.
Together they answer two engineering questions:

1. Does the running Gazebo/Nav2 system enter the expected latched fault and
   command zero when one required input disappears?
2. How long does the host-side command path take from a controlled injection
   boundary to the first zero command observed on `/cmd_vel_safe`?

These modules measure command-level reaction. They do not claim physical
stopping distance, certified functional safety, MCU watchdog performance, or
real-sensor communication latency.

## Isolation from Normal Navigation

The normal `benchmark.launch.py` defaults remain unchanged. Fault injection is
enabled only by `run_safety_benchmark.launch.py`, which passes a dedicated
supervisor parameter file and starts the test runner.

The dedicated profile changes only these safety evidence inputs:

```text
/scan               -> test relay -> /safety_benchmark/scan
/odometry/filtered  -> test relay -> /safety_benchmark/odometry
test lifecycle services          -> /safety_benchmark/*/is_active
```

The velocity path remains the M7 production path:

```text
/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel_collision_checked
  -> safety_supervisor -> /cmd_vel_safe
  -> /diff_drive_base_controller/cmd_vel
```

The runner may create a second `/cmd_vel_collision_checked` publisher only
during the explicit reset-recovery phase, after the Nav2 goal is canceled. It
publishes zero only, then destroys that publisher before the next trial. This
is test instrumentation, never part of a normal launch, and the runtime graph
gate must pass again after the runner exits.

## M7.5: Controlled Fault Injection

### Module purpose

M7.5 supplies deterministic boundaries for four failure classes while the
robot is receiving a nonzero Nav2 command:

| Scenario | Injection | Expected fault | Acceptance limit |
| --- | --- | --- | ---: |
| `estop` | Publish retained `true` | `MANUAL_ESTOP` | 40 ms |
| `scan_timeout` | Stop forwarding scans | `SCAN_STALE` | 540 ms |
| `odom_timeout` | Stop forwarding filtered odometry | `ODOM_STALE` | 540 ms |
| `nav2_timeout` | Remove the navigation health proxy service | `NAV2_INACTIVE` | 1040 ms |

The timeout limits are the configured watchdog deadline plus two 50 Hz safety
cycles. They were defined before measurement in M7.3-M7.4.

Each trial follows the same state machine:

1. Verify the real Nav2 lifecycle nodes and ros2_control controllers are active.
2. Wait for the supervisor to report `READY` with no faults.
3. Send a navigation goal and require a nonzero `/cmd_vel_safe` sample.
4. Record `time.monotonic_ns()` and inject exactly one fault.
5. Record the first matching `SafetyStatus` and first zero safe command.
6. Cancel the navigation goal and restore the injected evidence source.
7. Supply a test-only fresh zero stream for the guarded reset handshake.
8. Require `reset accepted` and a clean `READY` state before another trial.
9. Append one CSV row, flush it, and call `fsync` before proceeding.

If arming, injection, observation, cancellation, or guarded reset fails, the
runner records the failure and does not begin another motion trial.

### Safety constraints

- A trial cannot inject a fault until a finite nonzero safe command was
  observed recently.
- Only one fault source is disabled per trial.
- Gates default to forwarding; an unexpected runner exit cannot modify the
  normal launch configuration.
- All timing uses the steady monotonic clock, not simulation time.
- Raw input messages retain their original headers; the relays measure receipt
  freshness, matching the supervisor contract.
- The test runner never publishes to the controller command topic.

## M7.6: Durable Measurement and Offline Gate

### CSV evidence

Each row stores the run/trial identity, scenario, configured timeout and
predeclared acceptance limit, injection/status/zero monotonic timestamps,
derived fault-detection and zero-command latencies, the pre-fault command,
observed state and fault masks, relevant input age, reset result, trial result,
and failure reason.

The runner creates the CSV header before motion. Every completed row is flushed
and `fsync`ed so an unrelated later crash does not erase earlier evidence.

### Offline validation

`summarize_safety_benchmark.py` is independent from ROS graph state. It rejects:

- missing or extra scenario repetitions;
- unsupported schemas, duplicate trial indices, or multiple run IDs per file;
- non-finite, negative, or inconsistent timestamps and derived latencies;
- a zero-output event recorded before injection;
- a trial without a recent finite nonzero pre-fault command;
- the wrong state/fault bit, unsuccessful guarded reset, or nonzero output;
- a measured zero-command latency above the predeclared scenario limit;
- a CSV `trial_pass` value inconsistent with the stored evidence.

The tool produces Markdown and JSON reports using atomic replacement. Reports
include per-scenario count, success rate, mean, population standard deviation,
minimum, median, P95, and maximum command-level latency.

### Acceptance run

The first formal M7.6 run uses three repetitions of all four scenarios (12
trials) in one Gazebo/Nav2 process. Acceptance requires:

- all 12 trials present and valid;
- every trial observes the expected fault and a zero command;
- every trial is within its predeclared limit;
- every guarded reset succeeds before the next trial;
- no duplicate downstream publisher remains after the test;
- the existing four-package regression suite remains green.

Any failed formal run is retained and reported; thresholds are not widened to
hide a failure.

## Implementation and Acceptance Result (2026-09-27)

M7.5 and M7.6 are implemented. `run_safety_benchmark.launch.py` starts the M6
Gazebo/Nav2 stack with the isolated safety profile and launches
`safety_fault_injection_runner.py`. A dry run verified all 12 lifecycle nodes,
both ros2_control controllers, the navigation action server, source relays, and
a clean supervisor `READY` state without commanding motion.

Before the formal run, a one-scenario E-stop smoke test and a four-scenario
smoke test both passed. The formal run then executed three complete repetitions
in one Gazebo/Nav2 process. All 12 trials observed a finite nonzero safe command
before injection, entered the expected latched state, observed a zero safe
command within the predeclared limit, canceled the Nav2 goal, restored the
fault source, and completed the guarded reset before the next trial.

| Scenario | Pass | Limit (ms) | Mean (ms) | P95 (ms) | Max (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| `estop` | 3/3 | 40.0 | 0.815 | 0.967 | 0.977 |
| `scan_timeout` | 3/3 | 540.0 | 466.520 | 497.954 | 500.000 |
| `odom_timeout` | 3/3 | 540.0 | 500.131 | 500.242 | 500.251 |
| `nav2_timeout` | 3/3 | 1040.0 | 878.260 | 948.506 | 958.510 |

The independent analyzer recomputed every derived latency and returned
evidence validation `PASS`, acceptance `PASS`, 12/12 successful trials, and no
warnings. Evidence is retained in:

- `benchmark_results/safety_benchmark_20260927.csv`
- `benchmark_results/safety_benchmark_20260927.json`
- `benchmark_results/safety_benchmark_20260927.md`

The final affected-package regression suite reported `138 tests, 0 errors, 0
failures, 9 skipped`. The skipped checks are the existing Jazzy handling for a
known slow cppcheck version, not failed functional tests.

Gazebo Harmonic still prints a known shutdown-only segmentation fault after
`/server_control` has returned `data: true`. The runner exits cleanly and the
CSV is flushed and `fsync`ed before shutdown; process inspection shows no
remaining simulation backend. This is tracked as a shutdown compatibility
defect and is not counted as a successful safety response.

The reported latency is host-side command-level evidence only. It does not
claim physical braking performance, stopping distance, MCU watchdog response,
or certified functional safety.
