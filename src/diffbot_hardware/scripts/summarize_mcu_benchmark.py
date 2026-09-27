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

"""Validate M8.5 CSV evidence and generate JSON/Markdown reports."""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import statistics


SCENARIOS = ('drop_commands', 'drop_states', 'command_delay', 'reboot')
LIMIT_MS = 260.0
REQUIRED_FIELDS = (
    'schema_version', 'run_id', 'repetition', 'trial_index',
    'fault_name', 'configured_heartbeat_timeout_ms', 'command_delay_ms',
    'acceptance_limit_ms', 'injection_monotonic_ns',
    'fault_status_monotonic_ns', 'zero_output_monotonic_ns',
    'fault_detection_latency_ms', 'zero_command_latency_ms',
    'pre_fault_linear_x', 'observed_state', 'observed_active_faults',
    'observed_latched_faults', 'trial_pass', 'failure_reason',
)


def parse_bool(value):
    """Parse an unambiguous CSV boolean."""
    if value == 'True':
        return True
    if value == 'False':
        return False
    raise ValueError(f'invalid boolean: {value!r}')


def percentile(values, fraction):
    """Return a linearly interpolated percentile."""
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (
        position - lower
    )


def load_rows(path):
    """Load typed rows while preserving schema errors."""
    rows = []
    errors = []
    with Path(path).open(newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        missing = [
            field for field in REQUIRED_FIELDS
            if field not in (reader.fieldnames or [])
        ]
        if missing:
            return [], [f'missing fields: {missing}']
        for line, raw in enumerate(reader, 2):
            try:
                row = dict(raw)
                for field in (
                    'schema_version', 'repetition', 'trial_index',
                    'command_delay_ms', 'injection_monotonic_ns',
                    'fault_status_monotonic_ns', 'zero_output_monotonic_ns',
                    'observed_state', 'observed_active_faults',
                    'observed_latched_faults',
                ):
                    row[field] = int(raw[field])
                for field in (
                    'configured_heartbeat_timeout_ms',
                    'acceptance_limit_ms', 'fault_detection_latency_ms',
                    'zero_command_latency_ms', 'pre_fault_linear_x',
                ):
                    row[field] = float(raw[field])
                row['trial_pass'] = parse_bool(raw['trial_pass'])
                row['_line'] = line
                rows.append(row)
            except (TypeError, ValueError) as error:
                errors.append(f'line {line}: {error}')
    return rows, errors


def analyze(path, expected_repetitions=3):
    """Recompute acceptance without trusting stored trial flags."""
    rows, errors = load_rows(path)
    counts = Counter()
    run_ids = set()
    indices = []
    for row in rows:
        reference = f"line {row['_line']}"
        run_ids.add(row['run_id'])
        indices.append(row['trial_index'])
        counts[(row['repetition'], row['fault_name'])] += 1
        if row['schema_version'] != 1:
            errors.append(f'{reference}: unsupported schema version')
        if row['fault_name'] not in SCENARIOS:
            errors.append(f'{reference}: unknown fault')
            continue
        if row['configured_heartbeat_timeout_ms'] != 200.0:
            errors.append(f'{reference}: heartbeat timeout changed')
        if row['acceptance_limit_ms'] != LIMIT_MS:
            errors.append(f'{reference}: acceptance limit changed')
        start = row['injection_monotonic_ns']
        fault = row['fault_status_monotonic_ns']
        zero = row['zero_output_monotonic_ns']
        expected_fault = (fault - start) / 1.0e6
        expected_zero = (zero - start) / 1.0e6
        evidence_pass = (
            start > 0 and fault >= start and zero >= start
            and abs(row['fault_detection_latency_ms'] - expected_fault) < 0.02
            and abs(row['zero_command_latency_ms'] - expected_zero) < 0.02
            and row['zero_command_latency_ms'] <= LIMIT_MS
            and abs(row['pre_fault_linear_x']) > 1.0e-4
            and row['observed_state'] == 2
            and row['observed_active_faults'] & 128
            and row['observed_latched_faults'] & 128
        )
        if bool(evidence_pass) != row['trial_pass']:
            errors.append(f'{reference}: trial_pass disagrees with evidence')
        if row['trial_pass'] and row['failure_reason']:
            errors.append(f'{reference}: passing row has failure_reason')
    expected_total = expected_repetitions * len(SCENARIOS)
    if len(run_ids) != 1:
        errors.append('expected exactly one run_id')
    if sorted(indices) != list(range(1, expected_total + 1)):
        errors.append('trial_index sequence is incomplete')
    for repetition in range(1, expected_repetitions + 1):
        for scenario in SCENARIOS:
            if counts[(repetition, scenario)] != 1:
                errors.append(
                    f'repetition {repetition} scenario {scenario} missing'
                )

    summaries = {}
    for scenario in SCENARIOS:
        values = [
            row['zero_command_latency_ms'] for row in rows
            if row['fault_name'] == scenario and row['trial_pass']
        ]
        summaries[scenario] = {
            'trials': sum(row['fault_name'] == scenario for row in rows),
            'successes': len(values),
            'mean_ms': statistics.fmean(values) if values else None,
            'p95_ms': percentile(values, 0.95) if values else None,
            'max_ms': max(values) if values else None,
        }
    valid = not errors
    accepted = valid and len(rows) == expected_total and all(
        row['trial_pass'] for row in rows
    )
    return {
        'schema_version': 1,
        'source_csv': str(Path(path).resolve()),
        'valid': valid,
        'accepted': accepted,
        'errors': errors,
        'overall': {
            'trials': len(rows),
            'successes': sum(row['trial_pass'] for row in rows),
            'acceptance_limit_ms': LIMIT_MS,
        },
        'scenarios': summaries,
    }


def render_markdown(report):
    """Render the machine-validated report as Markdown."""
    lines = [
        '# M8 Virtual-MCU Transport Fault Benchmark', '',
        f"Acceptance: **{'PASS' if report['accepted'] else 'FAIL'}**", '',
        '| Scenario | Success | Mean (ms) | P95 (ms) | Max (ms) |',
        '| --- | ---: | ---: | ---: | ---: |',
    ]
    for name in SCENARIOS:
        item = report['scenarios'][name]

        def format_value(value):
            return 'n/a' if value is None else f'{value:.3f}'

        lines.append(
            f"| {name} | {item['successes']}/{item['trials']} | "
            f"{format_value(item['mean_ms'])} | "
            f"{format_value(item['p95_ms'])} | "
            f"{format_value(item['max_ms'])} |"
        )
    lines.extend([
        '',
        'Limit: 260 ms from the host fault-service call boundary to the '
        'first zero observed on `/cmd_vel_safe`.',
        '',
        'This does not measure physical wheel deceleration or stopping distance.',
    ])
    if report['errors']:
        lines.extend(['', '## Validation errors', ''])
        lines.extend(f'- {error}' for error in report['errors'])
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_path')
    parser.add_argument('--expected-repetitions', type=int, default=3)
    arguments = parser.parse_args()
    report = analyze(arguments.csv_path, arguments.expected_repetitions)
    base = Path(arguments.csv_path).with_suffix('')
    json_path = base.with_suffix('.json')
    markdown_path = base.with_suffix('.md')
    json_path.write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8'
    )
    markdown_path.write_text(render_markdown(report), encoding='utf-8')
    print(markdown_path)
    raise SystemExit(0 if report['accepted'] else 1)


if __name__ == '__main__':
    main()
