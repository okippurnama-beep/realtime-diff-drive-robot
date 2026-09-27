#!/usr/bin/env python3
# Copyright 2026 xiayuru
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run each M8.5 fault in a fresh ROS stack and generate reports."""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import sys


SCENARIOS = ('drop_commands', 'drop_states', 'command_delay', 'reboot')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--output', default='')
    arguments = parser.parse_args()
    if arguments.repetitions < 1:
        parser.error('--repetitions must be positive')
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    output = (
        Path(arguments.output).expanduser().resolve()
        if arguments.output
        else Path.cwd() / 'benchmark_results'
        / f'mcu_transport_benchmark_{run_id}.csv'
    )
    if output.exists():
        parser.error(f'refusing to overwrite existing evidence: {output}')
    output.parent.mkdir(parents=True, exist_ok=True)

    trial_index = 0
    for repetition in range(1, arguments.repetitions + 1):
        for scenario in SCENARIOS:
            trial_index += 1
            command = [
                'ros2', 'launch', 'robot_description',
                'mcu_fault_trial.launch.py',
                f'scenario:={scenario}',
                f'repetition:={repetition}',
                f'trial_index:={trial_index}',
                f'run_id:={run_id}',
                f'output_csv:={output}',
            ]
            print(
                f'[{trial_index}/{arguments.repetitions * len(SCENARIOS)}] '
                f'{scenario}',
                flush=True,
            )
            result = subprocess.run(
                command,
                check=False,
                timeout=30,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            if result.returncode != 0:
                print(result.stdout[-4000:], file=sys.stderr)
                raise RuntimeError(
                    f'launch failed for {scenario}: {result.returncode}'
                )

    summary = Path(__file__).with_name('summarize_mcu_benchmark.py')
    result = subprocess.run(
        [
            sys.executable,
            str(summary),
            str(output),
            '--expected-repetitions',
            str(arguments.repetitions),
        ],
        check=False,
    )
    print(f'Raw evidence: {output}')
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
