# Safety Fault-Injection Benchmark Summary

- Evidence validation: **PASS**
- Acceptance: **PASS**
- Runs: 1
- Trials: 12
- Successful trials: 12
- Success rate: 100.00%

## Per-scenario command-level latency

| Scenario | Trials | Pass | Rate | Limit (ms) | Mean (ms) | P95 (ms) | Max (ms) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `estop` | 3 | 3 | 100.00% | 40.0 | 0.815 | 0.967 | 0.977 |
| `scan_timeout` | 3 | 3 | 100.00% | 540.0 | 466.520 | 497.954 | 500.000 |
| `odom_timeout` | 3 | 3 | 100.00% | 540.0 | 500.131 | 500.242 | 500.251 |
| `nav2_timeout` | 3 | 3 | 100.00% | 1040.0 | 878.260 | 948.506 | 958.510 |

## Validation details

- No validation errors or warnings.

## Trial failure classification

- No failed trials.

These are host-side command-level measurements. They are not physical stopping-distance or functional-safety certification data.
