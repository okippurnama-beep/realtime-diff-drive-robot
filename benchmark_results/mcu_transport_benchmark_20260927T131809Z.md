# M8 Virtual-MCU Transport Fault Benchmark

Acceptance: **PASS**

| Scenario | Success | Mean (ms) | P95 (ms) | Max (ms) |
| --- | ---: | ---: | ---: | ---: |
| drop_commands | 3/3 | 114.864 | 117.747 | 117.939 |
| drop_states | 3/3 | 206.523 | 217.905 | 219.912 |
| command_delay | 3/3 | 115.292 | 119.250 | 119.259 |
| reboot | 3/3 | 4.930 | 8.495 | 8.962 |

Limit: 260 ms from the host fault-service call boundary to the first zero observed on `/cmd_vel_safe`.

This does not measure physical wheel deceleration or stopping distance.
