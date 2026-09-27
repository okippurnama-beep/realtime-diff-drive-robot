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

"""Unit tests for the offline M7.6 safety benchmark evidence gate."""

import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'summarize_safety_benchmark.py'
)
SPEC = importlib.util.spec_from_file_location(
    'summarize_safety_benchmark',
    SCRIPT_PATH,
)
SUMMARY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUMMARY)


class SafetyBenchmarkSummaryTest(unittest.TestCase):
    """Exercise valid, failed, incomplete, and tampered evidence."""

    def setUp(self):
        self._temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self._temporary_directory.cleanup)
        self._directory = Path(self._temporary_directory.name)

    @staticmethod
    def _row(fault_name, trial_index):
        scenario = SUMMARY.SCENARIOS[fault_name]
        injection_ns = 1_000_000_000 + trial_index * 2_000_000_000
        latency_ms = (
            10.0 if fault_name == 'estop'
            else scenario['timeout_ms'] + 10.0
        )
        event_ns = injection_ns + int(latency_ms * 1.0e6)
        input_age = (
            -1.0 if fault_name in ('estop', 'nav2_timeout')
            else scenario['timeout_ms'] / 1000.0
        )
        return {
            'schema_version': 1,
            'run_id': '20260927T120000Z',
            'repetition': 1,
            'trial_index': trial_index,
            'fault_name': fault_name,
            'expected_fault_bit': scenario['fault_bit'],
            'configured_timeout_ms': scenario['timeout_ms'],
            'acceptance_limit_ms': scenario['limit_ms'],
            'injection_monotonic_ns': injection_ns,
            'fault_status_monotonic_ns': event_ns,
            'zero_output_monotonic_ns': event_ns,
            'fault_detection_latency_ms': latency_ms,
            'zero_command_latency_ms': latency_ms,
            'pre_fault_linear_x': 0.2,
            'pre_fault_angular_z': 0.0,
            'observed_state': scenario['state'],
            'observed_active_faults': scenario['fault_bit'],
            'observed_latched_faults': scenario['fault_bit'],
            'observed_input_age_sec': input_age,
            'zero_output_observed': True,
            'reset_success': True,
            'reset_message': 'reset accepted',
            'trial_pass': True,
            'failure_reason': '',
        }

    def _valid_rows(self):
        return [
            self._row(name, index)
            for index, name in enumerate(
                SUMMARY.DEFAULT_SCENARIO_ORDER,
                start=1,
            )
        ]

    def _write_csv(self, rows, name='safety.csv'):
        path = self._directory / name
        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=SUMMARY.REQUIRED_FIELDS,
            )
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_accepts_complete_consistent_evidence(self):
        path = self._write_csv(self._valid_rows())
        report = SUMMARY.analyze_results(
            [path],
            expected_repetitions=1,
        )
        self.assertTrue(report['valid'])
        self.assertTrue(report['accepted'])
        self.assertEqual(report['overall']['trials'], 4)
        self.assertIn('Acceptance: **PASS**', SUMMARY.render_markdown(report))

    def test_preserves_valid_failed_trial_without_accepting_run(self):
        rows = self._valid_rows()
        row = rows[0]
        latency_ms = 50.0
        row['zero_output_monotonic_ns'] = (
            row['injection_monotonic_ns'] + int(latency_ms * 1.0e6)
        )
        row['zero_command_latency_ms'] = latency_ms
        row['trial_pass'] = False
        row['failure_reason'] = 'zero-command latency exceeded limit'
        report = SUMMARY.analyze_results(
            [self._write_csv(rows)],
            expected_repetitions=1,
        )
        self.assertTrue(report['valid'])
        self.assertFalse(report['accepted'])
        self.assertEqual(report['overall']['successes'], 3)

    def test_rejects_missing_scenario_row(self):
        path = self._write_csv(self._valid_rows()[:-1])
        report = SUMMARY.analyze_results(
            [path],
            expected_repetitions=1,
        )
        self.assertFalse(report['valid'])
        self.assertFalse(report['accepted'])
        self.assertTrue(any(
            'trial_index sequence' in error
            or 'scenarios are' in error
            for error in report['validation']['errors']
        ))

    def test_rejects_tampered_derived_latency(self):
        rows = self._valid_rows()
        rows[1]['zero_command_latency_ms'] += 20.0
        path = self._write_csv(rows)
        report = SUMMARY.analyze_results(
            [path],
            expected_repetitions=1,
        )
        self.assertFalse(report['valid'])
        self.assertTrue(any(
            'does not match timestamps' in error
            for error in report['validation']['errors']
        ))


if __name__ == '__main__':
    unittest.main()
