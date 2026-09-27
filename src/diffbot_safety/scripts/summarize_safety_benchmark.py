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

"""Validate safety benchmark CSV evidence and generate durable summaries."""

import argparse
from collections import Counter, defaultdict
import csv
import json
import math
import os
from pathlib import Path
import statistics
import sys
import tempfile


SCHEMA_VERSION = 1

REQUIRED_FIELDS = (
    'schema_version',
    'run_id',
    'repetition',
    'trial_index',
    'fault_name',
    'expected_fault_bit',
    'configured_timeout_ms',
    'acceptance_limit_ms',
    'injection_monotonic_ns',
    'fault_status_monotonic_ns',
    'zero_output_monotonic_ns',
    'fault_detection_latency_ms',
    'zero_command_latency_ms',
    'pre_fault_linear_x',
    'pre_fault_angular_z',
    'observed_state',
    'observed_active_faults',
    'observed_latched_faults',
    'observed_input_age_sec',
    'zero_output_observed',
    'reset_success',
    'reset_message',
    'trial_pass',
    'failure_reason',
)

INTEGER_FIELDS = (
    'schema_version',
    'repetition',
    'trial_index',
    'expected_fault_bit',
    'injection_monotonic_ns',
    'fault_status_monotonic_ns',
    'zero_output_monotonic_ns',
    'observed_state',
    'observed_active_faults',
    'observed_latched_faults',
)

FLOAT_FIELDS = (
    'configured_timeout_ms',
    'acceptance_limit_ms',
    'fault_detection_latency_ms',
    'zero_command_latency_ms',
    'pre_fault_linear_x',
    'pre_fault_angular_z',
    'observed_input_age_sec',
)

BOOLEAN_FIELDS = (
    'zero_output_observed',
    'reset_success',
    'trial_pass',
)

SCENARIOS = {
    'estop': {
        'fault_bit': 1,
        'state': 3,
        'timeout_ms': 0.0,
        'limit_ms': 40.0,
    },
    'scan_timeout': {
        'fault_bit': 4,
        'state': 2,
        'timeout_ms': 500.0,
        'limit_ms': 540.0,
    },
    'odom_timeout': {
        'fault_bit': 8,
        'state': 2,
        'timeout_ms': 500.0,
        'limit_ms': 540.0,
    },
    'nav2_timeout': {
        'fault_bit': 16,
        'state': 2,
        'timeout_ms': 1000.0,
        'limit_ms': 1040.0,
    },
}

DEFAULT_SCENARIO_ORDER = tuple(SCENARIOS)


def parse_boolean(value):
    """Parse a CSV boolean without accepting ambiguous spellings."""
    normalized = value.strip().lower()
    if normalized == 'true':
        return True
    if normalized == 'false':
        return False
    raise ValueError(f'expected True or False, got {value!r}')


def parse_row(raw_row, source, line_number):
    """Convert one raw CSV row into typed evidence."""
    row = dict(raw_row)
    row['_source'] = str(source)
    row['_line'] = line_number
    for field in INTEGER_FIELDS:
        row[field] = int(raw_row[field])
    for field in FLOAT_FIELDS:
        row[field] = float(raw_row[field])
    for field in BOOLEAN_FIELDS:
        row[field] = parse_boolean(raw_row[field])
    return row


def load_csv_files(paths):
    """Load CSV inputs and collect schema/parsing problems."""
    records = []
    errors = []
    warnings = []
    for path in (Path(item).expanduser().resolve() for item in paths):
        try:
            with path.open(newline='', encoding='utf-8') as stream:
                reader = csv.DictReader(stream)
                fields = reader.fieldnames or []
                missing = [field for field in REQUIRED_FIELDS
                           if field not in fields]
                extra = [field for field in fields
                         if field not in REQUIRED_FIELDS]
                if missing:
                    errors.append(
                        f'{path}: missing CSV fields: {", ".join(missing)}'
                    )
                    continue
                if extra:
                    warnings.append(
                        f'{path}: ignored extra CSV fields: '
                        f'{", ".join(extra)}'
                    )
                before = len(records)
                for line_number, raw_row in enumerate(reader, start=2):
                    try:
                        records.append(
                            parse_row(raw_row, path, line_number)
                        )
                    except (TypeError, ValueError) as error:
                        errors.append(f'{path}:{line_number}: {error}')
                if len(records) == before:
                    errors.append(f'{path}: contains no valid data rows')
        except (OSError, csv.Error) as error:
            errors.append(f'{path}: {error}')
    return records, errors, warnings


def row_reference(row):
    """Return a stable source location for an evidence row."""
    return f"{row['_source']}:{row['_line']}"


def close_enough(actual, expected, tolerance=1.0e-3):
    """Compare values while tolerating CSV float serialization noise."""
    return math.isfinite(actual) and abs(actual - expected) <= tolerance


def trial_evidence_passes(row):
    """Recompute the trial result without trusting the stored pass flag."""
    scenario = SCENARIOS.get(row['fault_name'])
    if scenario is None:
        return False
    pre_fault_nonzero = (
        math.isfinite(row['pre_fault_linear_x'])
        and math.isfinite(row['pre_fault_angular_z'])
        and (
            abs(row['pre_fault_linear_x']) > 1.0e-4
            or abs(row['pre_fault_angular_z']) > 1.0e-4
        )
    )
    return (
        row['observed_state'] == scenario['state']
        and row['observed_active_faults'] & scenario['fault_bit'] != 0
        and row['observed_latched_faults'] & scenario['fault_bit'] != 0
        and row['zero_output_observed']
        and math.isfinite(row['zero_command_latency_ms'])
        and 0.0 <= row['zero_command_latency_ms']
        <= scenario['limit_ms']
        and pre_fault_nonzero
        and row['reset_success']
        and row['reset_message'] == 'reset accepted'
    )


def validate_row(row):
    """Validate one row's schema, timestamps, and derived evidence."""
    errors = []
    reference = row_reference(row)
    if row['schema_version'] != SCHEMA_VERSION:
        errors.append(
            f'{reference}: unsupported schema_version '
            f"{row['schema_version']}"
        )
    if not row['run_id'].strip():
        errors.append(f'{reference}: run_id must not be empty')
    if row['repetition'] < 1 or row['trial_index'] < 1:
        errors.append(
            f'{reference}: repetition and trial_index must be positive'
        )

    scenario = SCENARIOS.get(row['fault_name'])
    if scenario is None:
        errors.append(
            f"{reference}: unknown fault_name {row['fault_name']!r}"
        )
        return errors
    if row['expected_fault_bit'] != scenario['fault_bit']:
        errors.append(f'{reference}: expected_fault_bit does not match design')
    if not close_enough(
        row['configured_timeout_ms'], scenario['timeout_ms']
    ):
        errors.append(
            f'{reference}: configured_timeout_ms does not match design'
        )
    if not close_enough(
        row['acceptance_limit_ms'], scenario['limit_ms']
    ):
        errors.append(
            f'{reference}: acceptance_limit_ms does not match design'
        )

    injection_ns = row['injection_monotonic_ns']
    fault_ns = row['fault_status_monotonic_ns']
    zero_ns = row['zero_output_monotonic_ns']
    if injection_ns <= 0:
        errors.append(f'{reference}: injection timestamp must be positive')
    if fault_ns < injection_ns:
        errors.append(f'{reference}: fault status precedes injection')
    if zero_ns < injection_ns:
        errors.append(f'{reference}: zero output precedes injection')

    expected_fault_latency = (fault_ns - injection_ns) / 1.0e6
    expected_zero_latency = (zero_ns - injection_ns) / 1.0e6
    if not close_enough(
        row['fault_detection_latency_ms'],
        expected_fault_latency,
        0.01,
    ):
        errors.append(
            f'{reference}: fault_detection_latency_ms does not match '
            'timestamps'
        )
    if not close_enough(
        row['zero_command_latency_ms'],
        expected_zero_latency,
        0.01,
    ):
        errors.append(
            f'{reference}: zero_command_latency_ms does not match timestamps'
        )
    if not math.isfinite(row['observed_input_age_sec']):
        errors.append(
            f'{reference}: observed_input_age_sec must be finite'
        )

    recomputed_pass = trial_evidence_passes(row)
    if row['trial_pass'] != recomputed_pass:
        errors.append(
            f'{reference}: trial_pass disagrees with recorded evidence'
        )
    if row['trial_pass'] and row['failure_reason'].strip():
        errors.append(
            f'{reference}: passing trial has a failure_reason'
        )
    if not row['trial_pass'] and not row['failure_reason'].strip():
        errors.append(
            f'{reference}: failed trial must explain failure_reason'
        )
    return errors


def validate_run_groups(records, scenario_order, expected_repetitions):
    """Verify each run has one ordered row per requested scenario."""
    errors = []
    grouped = defaultdict(list)
    file_run_ids = defaultdict(set)
    for row in records:
        key = (row['_source'], row['run_id'])
        grouped[key].append(row)
        file_run_ids[row['_source']].add(row['run_id'])

    for source, run_ids in file_run_ids.items():
        if len(run_ids) != 1:
            errors.append(
                f'{source}: expected one run_id, found {len(run_ids)}'
            )
    seen_run_ids = defaultdict(set)
    for source, run_id in grouped:
        seen_run_ids[run_id].add(source)
    for run_id, sources in seen_run_ids.items():
        if len(sources) > 1:
            errors.append(
                f'run_id {run_id!r} appears in multiple input files'
            )

    expected_count = expected_repetitions * len(scenario_order)
    for (source, run_id), rows in grouped.items():
        ordered = sorted(rows, key=lambda item: item['trial_index'])
        indices = [row['trial_index'] for row in ordered]
        if indices != list(range(1, expected_count + 1)):
            errors.append(
                f'{source}: run {run_id} trial_index sequence is {indices}, '
                f'expected 1..{expected_count}'
            )
        for repetition in range(1, expected_repetitions + 1):
            names = [
                row['fault_name'] for row in ordered
                if row['repetition'] == repetition
            ]
            if names != scenario_order:
                errors.append(
                    f'{source}: run {run_id} repetition {repetition} '
                    f'scenarios are {names}, expected {scenario_order}'
                )
    return errors


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
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def descriptive_statistics(values):
    """Return stable statistics for finite measurements."""
    finite = [float(value) for value in values if math.isfinite(value)]
    if not finite:
        return {
            'count': 0,
            'mean': None,
            'stddev': None,
            'min': None,
            'p50': None,
            'p95': None,
            'max': None,
        }
    return {
        'count': len(finite),
        'mean': statistics.fmean(finite),
        'stddev': statistics.pstdev(finite),
        'min': min(finite),
        'p50': percentile(finite, 0.50),
        'p95': percentile(finite, 0.95),
        'max': max(finite),
    }


def summarize_rows(rows):
    """Compute pass counts and latency statistics for a row collection."""
    failure_counts = Counter(
        row['failure_reason'] for row in rows if not row['trial_pass']
    )
    total = len(rows)
    successes = sum(row['trial_pass'] for row in rows)
    return {
        'trials': total,
        'successes': successes,
        'success_rate_percent': 100.0 * successes / total if total else 0.0,
        'failure_counts': dict(sorted(failure_counts.items())),
        'fault_detection_latency_ms': descriptive_statistics(
            row['fault_detection_latency_ms'] for row in rows
        ),
        'zero_command_latency_ms': descriptive_statistics(
            row['zero_command_latency_ms'] for row in rows
        ),
    }


def analyze_results(csv_paths, expected_repetitions=3,
                    scenario_order=DEFAULT_SCENARIO_ORDER):
    """Validate CSV files and return a machine-readable report."""
    if expected_repetitions < 1:
        raise ValueError('expected_repetitions must be at least 1')
    scenario_order = list(scenario_order)
    if not scenario_order:
        raise ValueError('scenario_order must not be empty')
    if len(scenario_order) != len(set(scenario_order)):
        raise ValueError('scenario_order must not contain duplicates')
    unknown = [name for name in scenario_order if name not in SCENARIOS]
    if unknown:
        raise ValueError(f'unknown scenarios: {unknown}')

    records, errors, warnings = load_csv_files(csv_paths)
    for row in records:
        errors.extend(validate_row(row))
    errors.extend(
        validate_run_groups(records, scenario_order, expected_repetitions)
    )

    overall = summarize_rows(records)
    valid = not errors
    accepted = valid and overall['trials'] > 0 and (
        overall['successes'] == overall['trials']
    )
    return {
        'report_schema_version': 1,
        'valid': valid,
        'accepted': accepted,
        'validation': {
            'errors': errors,
            'warnings': warnings,
            'expected_repetitions_per_run': expected_repetitions,
            'scenario_order': scenario_order,
        },
        'inputs': [str(Path(path).expanduser().resolve())
                   for path in csv_paths],
        'run_ids': sorted({row['run_id'] for row in records}),
        'run_count': len({row['run_id'] for row in records}),
        'overall': overall,
        'per_scenario': {
            name: summarize_rows(
                [row for row in records if row['fault_name'] == name]
            )
            for name in scenario_order
        },
    }


def format_number(value, digits=3):
    """Format an optional numeric report value."""
    return 'n/a' if value is None else f'{value:.{digits}f}'


def render_markdown(report):
    """Render a reviewable Markdown acceptance report."""
    overall = report['overall']
    evidence = 'PASS' if report['valid'] else 'FAIL'
    acceptance = 'PASS' if report['accepted'] else 'FAIL'
    lines = [
        '# Safety Fault-Injection Benchmark Summary',
        '',
        f'- Evidence validation: **{evidence}**',
        f'- Acceptance: **{acceptance}**',
        f"- Runs: {report['run_count']}",
        f"- Trials: {overall['trials']}",
        f"- Successful trials: {overall['successes']}",
        f"- Success rate: {overall['success_rate_percent']:.2f}%",
        '',
        '## Per-scenario command-level latency',
        '',
        '| Scenario | Trials | Pass | Rate | Limit (ms) | Mean (ms) | '
        'P95 (ms) | Max (ms) |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |',
    ]
    for name, summary in report['per_scenario'].items():
        stats = summary['zero_command_latency_ms']
        lines.append(
            f'| `{name}` | {summary["trials"]} | '
            f'{summary["successes"]} | '
            f'{summary["success_rate_percent"]:.2f}% | '
            f'{SCENARIOS[name]["limit_ms"]:.1f} | '
            f'{format_number(stats["mean"])} | '
            f'{format_number(stats["p95"])} | '
            f'{format_number(stats["max"])} |'
        )

    lines.extend(['', '## Validation details', ''])
    errors = report['validation']['errors']
    warnings = report['validation']['warnings']
    if not errors and not warnings:
        lines.append('- No validation errors or warnings.')
    else:
        lines.extend(f'- ERROR: {message}' for message in errors)
        lines.extend(f'- WARNING: {message}' for message in warnings)

    lines.extend(['', '## Trial failure classification', ''])
    failures = overall['failure_counts']
    if failures:
        lines.extend(
            f'- `{reason}`: {count}'
            for reason, count in failures.items()
        )
    else:
        lines.append('- No failed trials.')
    lines.extend([
        '',
        'These are host-side command-level measurements. They are not '
        'physical stopping-distance or functional-safety certification data.',
        '',
    ])
    return '\n'.join(lines)


def atomic_write(path, content):
    """Atomically replace a report after flushing it to disk."""
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=destination.parent,
            prefix=f'.{destination.name}.',
            suffix='.tmp',
            delete=False,
        ) as stream:
            temporary_name = stream.name
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, destination)
    finally:
        if temporary_name is not None and os.path.exists(temporary_name):
            os.unlink(temporary_name)


def parse_arguments(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_files', nargs='+', help='Runner CSV file(s).')
    parser.add_argument(
        '--expected-repetitions',
        type=int,
        default=3,
        help='Expected complete repetitions in each input file.',
    )
    parser.add_argument(
        '--scenarios',
        default=','.join(DEFAULT_SCENARIO_ORDER),
        help='Expected comma-separated scenario order.',
    )
    parser.add_argument('--output-json', help='Optional JSON report path.')
    parser.add_argument(
        '--output-markdown',
        help='Optional Markdown report path.',
    )
    return parser.parse_args(argv)


def main(argv=None):
    """Validate evidence, write reports, and return the gate status."""
    arguments = parse_arguments(argv)
    try:
        scenario_order = [
            name.strip() for name in arguments.scenarios.split(',')
            if name.strip()
        ]
        report = analyze_results(
            arguments.csv_files,
            expected_repetitions=arguments.expected_repetitions,
            scenario_order=scenario_order,
        )
        markdown = render_markdown(report)
        if arguments.output_json:
            atomic_write(
                arguments.output_json,
                json.dumps(report, indent=2, sort_keys=True) + '\n',
            )
        if arguments.output_markdown:
            atomic_write(arguments.output_markdown, markdown)
        print(markdown)
        return 0 if report['accepted'] else 2
    except (OSError, ValueError, csv.Error) as error:
        print(f'safety benchmark analysis error: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
