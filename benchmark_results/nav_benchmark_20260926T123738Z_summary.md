# Nav2 Benchmark Summary

- Validation: **FAIL**
- Runs: 1
- Trials: 1
- Successes: 0
- Success rate: 0.00%

## Per-goal results

| Goal | Trials | Success | Rate | Mean time (s) | P95 time (s) | Mean pos. error (m) | Mean yaw error (rad) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `straight_east` | 1 | 0 | 0.00% | n/a | n/a | n/a | n/a |
| `east_detour` | 0 | 0 | 0.00% | n/a | n/a | n/a | n/a |
| `south_west_corridor` | 0 | 0 | 0.00% | n/a | n/a | n/a | n/a |
| `north_west_turn` | 0 | 0 | 0.00% | n/a | n/a | n/a | n/a |
| `north_east_corridor` | 0 | 0 | 0.00% | n/a | n/a | n/a | n/a |

## Validation details

- ERROR: /home/xiayuru/robot_ws/benchmark_results/nav_benchmark_20260926T123738Z.csv:2: return_status 'CANCELED' is not valid for this report
- ERROR: /home/xiayuru/robot_ws/benchmark_results/nav_benchmark_20260926T123738Z.csv: run 20260926T123812Z trial_index sequence is [1], expected [1, 2, 3, 4, 5]
- ERROR: /home/xiayuru/robot_ws/benchmark_results/nav_benchmark_20260926T123738Z.csv: run 20260926T123812Z repetition 1 goals are ['straight_east'], expected ['straight_east', 'east_detour', 'south_west_corridor', 'north_west_turn', 'north_east_corridor']

## Failure classification

- `RETURN_CANCELED`: 1
