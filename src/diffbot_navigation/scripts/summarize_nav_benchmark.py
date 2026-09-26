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

"""Validate Nav2 benchmark CSV files and generate reproducible summaries."""

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

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
import yaml


SCHEMA_VERSION = 1
REQUIRED_FIELDS = (
    'schema_version',
    'run_id',
    'repetition',
    'trial_index',
    'goal_name',
    'target_x_m',
    'target_y_m',
    'target_yaw_rad',
    'start_x_m',
    'start_y_m',
    'start_yaw_rad',
    'finish_x_m',
    'finish_y_m',
    'finish_yaw_rad',
    'position_error_m',
    'yaw_error_rad',
    'action_status',
    'status_code',
    'nav2_error_code',
    'nav2_error_msg',
    'navigation_time_s',
    'wall_time_s',
    'max_recoveries',
    'final_distance_remaining_m',
    'timed_out',
    'return_status',
)

INTEGER_FIELDS = (
    'schema_version',
    'repetition',
    'trial_index',
    'status_code',
    'nav2_error_code',
    'max_recoveries',
)

FINITE_FLOAT_FIELDS = (
    'target_x_m',
    'target_y_m',
    'target_yaw_rad',
    'start_x_m',
    'start_y_m',
    'start_yaw_rad',
    'finish_x_m',
    'finish_y_m',
    'finish_yaw_rad',
    'position_error_m',
    'yaw_error_rad',
    'wall_time_s',
)

OPTIONAL_FLOAT_FIELDS = (
    'navigation_time_s',
    'final_distance_remaining_m',
)


def normalize_angle(angle):
    """Normalize an angle to the closed-open interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def default_layout_path():
    """Resolve the installed layout, with a source-tree fallback."""
    try:
        share = Path(get_package_share_directory('diffbot_navigation'))
        return share / 'config' / 'nav_benchmark_layout.yaml'
    except (LookupError, PackageNotFoundError):
        return (
            Path(__file__).resolve().parents[1]
            / 'config'
            / 'nav_benchmark_layout.yaml'
        )


def load_layout(path):
    """Load the benchmark start pose and ordered goal list."""
    with Path(path).open(encoding='utf-8') as stream:
        layout = yaml.safe_load(stream)

    if layout.get('version') != 1:
        raise ValueError('layout version must be 1')
    if 'robot' not in layout or 'start' not in layout['robot']:
        raise ValueError('layout must define robot.start')
    if not layout.get('goals'):
        raise ValueError('layout must define at least one goal')

    names = [goal.get('name') for goal in layout['goals']]
    if any(not name for name in names) or len(names) != len(set(names)):
        raise ValueError('layout goal names must be non-empty and unique')
    return layout


def parse_boolean(value):
    """Parse the bool spelling written by Python's CSV conversion."""
    normalized = value.strip().lower()
    if normalized == 'true':
        return True
    if normalized == 'false':
        return False
    raise ValueError(f'expected True or False, got {value!r}')


def parse_row(raw_row, source, line_number):
    """Convert one CSV row to typed values with strict numeric checks."""
    row = dict(raw_row)
    row['_source'] = str(source)
    row['_line'] = line_number

    for field in INTEGER_FIELDS:
        row[field] = int(raw_row[field])
    for field in FINITE_FLOAT_FIELDS:
        value = float(raw_row[field])
        if not math.isfinite(value):
            raise ValueError(f'{field} must be finite')
        row[field] = value
    for field in OPTIONAL_FLOAT_FIELDS:
        row[field] = float(raw_row[field])
    row['timed_out'] = parse_boolean(raw_row['timed_out'])

    if not row['run_id'].strip():
        raise ValueError('run_id must not be empty')
    if not row['goal_name'].strip():
        raise ValueError('goal_name must not be empty')
    if not row['action_status'].strip():
        raise ValueError('action_status must not be empty')
    if not row['return_status'].strip():
        raise ValueError('return_status must not be empty')
    return row


def load_csv_files(paths):
    """Read all inputs, collecting schema and row errors."""
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
    """Return a compact source location for a parsed row."""
    return f"{row['_source']}:{row['_line']}"


def validate_row_values(row, goals, start, tolerances,
                        allow_disabled_return):
    """Validate row-level invariants and return error messages."""
    errors = []
    reference = row_reference(row)

    if row['schema_version'] != SCHEMA_VERSION:
        errors.append(
            f'{reference}: unsupported schema_version '
            f"{row['schema_version']}"
        )
    if row['repetition'] < 1 or row['trial_index'] < 1:
        errors.append(
            f'{reference}: repetition and trial_index must be positive'
        )
    if row['max_recoveries'] < 0:
        errors.append(f'{reference}: max_recoveries must not be negative')
    for field in ('position_error_m', 'yaw_error_rad', 'wall_time_s'):
        if row[field] < 0.0:
            errors.append(f'{reference}: {field} must not be negative')

    goal = goals.get(row['goal_name'])
    if goal is None:
        errors.append(
            f"{reference}: unknown goal_name {row['goal_name']!r}"
        )
    else:
        recorded_target = (
            row['target_x_m'],
            row['target_y_m'],
            row['target_yaw_rad'],
        )
        for actual, expected, axis in zip(
            recorded_target,
            goal['pose'],
            ('x', 'y', 'yaw'),
        ):
            difference = (
                abs(normalize_angle(actual - expected))
                if axis == 'yaw' else abs(actual - expected)
            )
            if difference > tolerances['target']:
                errors.append(
                    f'{reference}: target {axis} does not match layout for '
                    f"{row['goal_name']}"
                )

    start_error = math.hypot(
        row['start_x_m'] - start[0],
        row['start_y_m'] - start[1],
    )
    start_yaw_error = abs(normalize_angle(row['start_yaw_rad'] - start[2]))
    if start_error > tolerances['start_position']:
        errors.append(
            f'{reference}: start position error {start_error:.4f} m exceeds '
            f"{tolerances['start_position']:.4f} m"
        )
    if start_yaw_error > tolerances['start_yaw']:
        errors.append(
            f'{reference}: start yaw error {start_yaw_error:.4f} rad exceeds '
            f"{tolerances['start_yaw']:.4f} rad"
        )

    computed_position_error = math.hypot(
        row['target_x_m'] - row['finish_x_m'],
        row['target_y_m'] - row['finish_y_m'],
    )
    computed_yaw_error = abs(normalize_angle(
        row['target_yaw_rad'] - row['finish_yaw_rad']
    ))
    if abs(computed_position_error - row['position_error_m']) > 1.0e-6:
        errors.append(
            f'{reference}: position_error_m does not match target and finish'
        )
    if abs(computed_yaw_error - row['yaw_error_rad']) > 1.0e-6:
        errors.append(
            f'{reference}: yaw_error_rad does not match target and finish'
        )

    if row['action_status'] == 'SUCCEEDED':
        if row['status_code'] != 4:
            errors.append(
                f'{reference}: SUCCEEDED requires status_code 4'
            )
        if row['nav2_error_code'] != 0:
            errors.append(
                f'{reference}: SUCCEEDED requires nav2_error_code 0'
            )
        if row['timed_out']:
            errors.append(f'{reference}: SUCCEEDED row cannot be timed out')
        if not math.isfinite(row['navigation_time_s']):
            errors.append(
                f'{reference}: SUCCEEDED requires finite navigation_time_s'
            )
        if not math.isfinite(row['final_distance_remaining_m']):
            errors.append(
                f'{reference}: SUCCEEDED requires finite '
                'final_distance_remaining_m'
            )
    if (
        math.isfinite(row['final_distance_remaining_m'])
        and row['final_distance_remaining_m'] < 0.0
    ):
        errors.append(
            f'{reference}: final_distance_remaining_m must not be negative'
        )

    valid_returns = {'SUCCEEDED'}
    if allow_disabled_return:
        valid_returns.add('DISABLED')
    if row['return_status'] not in valid_returns:
        errors.append(
            f"{reference}: return_status {row['return_status']!r} is not "
            'valid for this report'
        )
    return errors


def validate_run_groups(records, goal_order, expected_repetitions):
    """Check run completeness, uniqueness, ordering, and goal coverage."""
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

    expected_count = expected_repetitions * len(goal_order)
    for (source, run_id), rows in grouped.items():
        ordered = sorted(rows, key=lambda row: row['trial_index'])
        actual_indices = [row['trial_index'] for row in ordered]
        expected_indices = list(range(1, expected_count + 1))
        if actual_indices != expected_indices:
            errors.append(
                f'{source}: run {run_id} trial_index sequence is '
                f'{actual_indices}, expected {expected_indices}'
            )

        for repetition in range(1, expected_repetitions + 1):
            repetition_rows = [
                row for row in ordered
                if row['repetition'] == repetition
            ]
            names = [row['goal_name'] for row in repetition_rows]
            if names != goal_order:
                errors.append(
                    f'{source}: run {run_id} repetition {repetition} goals '
                    f'are {names}, expected {goal_order}'
                )

        unexpected_repetitions = sorted({
            row['repetition'] for row in ordered
            if row['repetition'] > expected_repetitions
        })
        if unexpected_repetitions:
            errors.append(
                f'{source}: run {run_id} has unexpected repetitions '
                f'{unexpected_repetitions}'
            )
    return errors


def trial_succeeded(row, allow_disabled_return):
    """Apply the formal trial-success definition."""
    return (
        row['action_status'] == 'SUCCEEDED'
        and row['status_code'] == 4
        and row['nav2_error_code'] == 0
        and not row['timed_out']
        and (
            row['return_status'] == 'SUCCEEDED'
            or (
                allow_disabled_return
                and row['return_status'] == 'DISABLED'
            )
        )
    )


def failure_reason(row, allow_disabled_return):
    """Classify an unsuccessful trial with one primary reason."""
    if row['timed_out']:
        return 'TIMEOUT'
    if row['action_status'] != 'SUCCEEDED':
        return f"ACTION_{row['action_status']}"
    if row['status_code'] != 4:
        return f"STATUS_CODE_{row['status_code']}"
    if row['nav2_error_code'] != 0:
        return f"NAV2_ERROR_{row['nav2_error_code']}"
    if (
        row['return_status'] != 'SUCCEEDED'
        and not (
            allow_disabled_return
            and row['return_status'] == 'DISABLED'
        )
    ):
        return f"RETURN_{row['return_status']}"
    return 'INCONSISTENT_RESULT'


def percentile(values, fraction):
    """Calculate a linearly interpolated percentile."""
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
    """Return stable descriptive statistics for finite values."""
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


def summarize_rows(rows, allow_disabled_return):
    """Compute success and performance metrics for a row collection."""
    successful = [
        row for row in rows
        if trial_succeeded(row, allow_disabled_return)
    ]
    failures = Counter(
        failure_reason(row, allow_disabled_return)
        for row in rows
        if not trial_succeeded(row, allow_disabled_return)
    )
    total = len(rows)
    return {
        'trials': total,
        'successes': len(successful),
        'success_rate_percent': (
            100.0 * len(successful) / total if total else 0.0
        ),
        'failure_counts': dict(sorted(failures.items())),
        'successful_trial_metrics': {
            field: descriptive_statistics(row[field] for row in successful)
            for field in (
                'navigation_time_s',
                'wall_time_s',
                'position_error_m',
                'yaw_error_rad',
            )
        },
        'all_trial_recoveries': descriptive_statistics(
            row['max_recoveries'] for row in rows
        ),
    }


def analyze_results(csv_paths, layout_path, expected_repetitions=1,
                    allow_disabled_return=False,
                    start_position_tolerance_m=0.12,
                    start_yaw_tolerance_rad=0.15):
    """Validate inputs and return a machine-readable report dictionary."""
    if expected_repetitions < 1:
        raise ValueError('expected_repetitions must be at least 1')
    if start_position_tolerance_m < 0.0 or start_yaw_tolerance_rad < 0.0:
        raise ValueError('start tolerances must not be negative')

    layout = load_layout(layout_path)
    goal_order = [goal['name'] for goal in layout['goals']]
    goals = {goal['name']: goal for goal in layout['goals']}
    start = layout['robot']['start']
    records, errors, warnings = load_csv_files(csv_paths)
    tolerances = {
        'target': 1.0e-6,
        'start_position': start_position_tolerance_m,
        'start_yaw': start_yaw_tolerance_rad,
    }

    for row in records:
        errors.extend(
            validate_row_values(
                row,
                goals,
                start,
                tolerances,
                allow_disabled_return,
            )
        )
    errors.extend(
        validate_run_groups(records, goal_order, expected_repetitions)
    )

    run_ids = sorted({row['run_id'] for row in records})
    overall = summarize_rows(records, allow_disabled_return)
    per_goal = {
        name: summarize_rows(
            [row for row in records if row['goal_name'] == name],
            allow_disabled_return,
        )
        for name in goal_order
    }
    return {
        'report_schema_version': 1,
        'valid': not errors,
        'validation': {
            'errors': errors,
            'warnings': warnings,
            'expected_repetitions_per_run': expected_repetitions,
            'required_return_to_start': not allow_disabled_return,
            'start_position_tolerance_m': start_position_tolerance_m,
            'start_yaw_tolerance_rad': start_yaw_tolerance_rad,
        },
        'inputs': [str(Path(path).expanduser().resolve())
                   for path in csv_paths],
        'run_ids': run_ids,
        'run_count': len(run_ids),
        'overall': overall,
        'per_goal': per_goal,
    }


def format_number(value, digits=4):
    """Format optional numeric report fields for Markdown."""
    return 'n/a' if value is None else f'{value:.{digits}f}'


def render_markdown(report):
    """Render the report as a reviewable Markdown artifact."""
    overall = report['overall']
    validity = 'PASS' if report['valid'] else 'FAIL'
    lines = [
        '# Nav2 Benchmark Summary',
        '',
        f'- Validation: **{validity}**',
        f"- Runs: {report['run_count']}",
        f"- Trials: {overall['trials']}",
        f"- Successes: {overall['successes']}",
        f"- Success rate: {overall['success_rate_percent']:.2f}%",
        '',
        '## Per-goal results',
        '',
        '| Goal | Trials | Success | Rate | Mean time (s) | '
        'P95 time (s) | Mean pos. error (m) | Mean yaw error (rad) |',
        '| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |',
    ]
    for name, summary in report['per_goal'].items():
        time_stats = summary['successful_trial_metrics'][
            'navigation_time_s'
        ]
        position_stats = summary['successful_trial_metrics'][
            'position_error_m'
        ]
        yaw_stats = summary['successful_trial_metrics']['yaw_error_rad']
        lines.append(
            f"| `{name}` | {summary['trials']} | {summary['successes']} | "
            f"{summary['success_rate_percent']:.2f}% | "
            f"{format_number(time_stats['mean'], 3)} | "
            f"{format_number(time_stats['p95'], 3)} | "
            f"{format_number(position_stats['mean'])} | "
            f"{format_number(yaw_stats['mean'])} |"
        )

    lines.extend(['', '## Validation details', ''])
    errors = report['validation']['errors']
    warnings = report['validation']['warnings']
    if not errors and not warnings:
        lines.append('- No validation errors or warnings.')
    else:
        lines.extend(f'- ERROR: {message}' for message in errors)
        lines.extend(f'- WARNING: {message}' for message in warnings)

    lines.extend(['', '## Failure classification', ''])
    failures = overall['failure_counts']
    if failures:
        lines.extend(
            f'- `{name}`: {count}' for name, count in failures.items()
        )
    else:
        lines.append('- No failed trials.')
    lines.append('')
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
    """Parse command-line options."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_files', nargs='+', help='Runner CSV file(s).')
    parser.add_argument(
        '--layout',
        default=str(default_layout_path()),
        help='Benchmark layout YAML path.',
    )
    parser.add_argument(
        '--expected-repetitions',
        type=int,
        default=1,
        help='Expected complete five-goal repetitions in each CSV.',
    )
    parser.add_argument(
        '--allow-disabled-return',
        action='store_true',
        help='Allow targeted smoke tests that do not return to start.',
    )
    parser.add_argument(
        '--start-position-tolerance-m',
        type=float,
        default=0.12,
        help='Maximum measured start-position error for each trial.',
    )
    parser.add_argument(
        '--start-yaw-tolerance-rad',
        type=float,
        default=0.15,
        help='Maximum measured start-yaw error for each trial.',
    )
    parser.add_argument('--output-json', help='Optional JSON report path.')
    parser.add_argument(
        '--output-markdown',
        help='Optional Markdown report path.',
    )
    return parser.parse_args(argv)


def main(argv=None):
    """Run validation, write requested reports, and return a status code."""
    arguments = parse_arguments(argv)
    try:
        report = analyze_results(
            arguments.csv_files,
            arguments.layout,
            expected_repetitions=arguments.expected_repetitions,
            allow_disabled_return=arguments.allow_disabled_return,
            start_position_tolerance_m=(
                arguments.start_position_tolerance_m
            ),
            start_yaw_tolerance_rad=arguments.start_yaw_tolerance_rad,
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
        return 0 if report['valid'] else 2
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f'benchmark summary error: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
