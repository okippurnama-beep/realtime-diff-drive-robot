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

"""Unit tests for the M8.5 offline evidence gate."""

import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'summarize_mcu_benchmark.py'
)
SPEC = importlib.util.spec_from_file_location('m8_summary', SCRIPT)
SUMMARY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUMMARY)


class McuBenchmarkSummaryTest(unittest.TestCase):
    """Check complete, missing, and tampered evidence."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)

    @staticmethod
    def row(name, index):
        injection = 1_000_000_000 + index * 1_000_000_000
        latency = 210.0
        event = injection + int(latency * 1.0e6)
        return {
            'schema_version': 1,
            'run_id': 'm8-test',
            'repetition': 1,
            'trial_index': index,
            'fault_name': name,
            'configured_heartbeat_timeout_ms': 200.0,
            'command_delay_ms': 150 if name == 'command_delay' else 0,
            'acceptance_limit_ms': 260.0,
            'injection_monotonic_ns': injection,
            'fault_status_monotonic_ns': event,
            'zero_output_monotonic_ns': event,
            'fault_detection_latency_ms': latency,
            'zero_command_latency_ms': latency,
            'pre_fault_linear_x': 0.12,
            'observed_state': 2,
            'observed_active_faults': 128,
            'observed_latched_faults': 128,
            'trial_pass': True,
            'failure_reason': '',
        }

    def write(self, rows):
        path = self.directory / 'evidence.csv'
        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(
                stream, fieldnames=SUMMARY.REQUIRED_FIELDS
            )
            writer.writeheader()
            writer.writerows(rows)
        return path

    def valid_rows(self):
        return [
            self.row(name, index)
            for index, name in enumerate(SUMMARY.SCENARIOS, 1)
        ]

    def test_accepts_complete_evidence(self):
        report = SUMMARY.analyze(self.write(self.valid_rows()), 1)
        self.assertTrue(report['valid'])
        self.assertTrue(report['accepted'])
        self.assertIn('Acceptance: **PASS**', SUMMARY.render_markdown(report))

    def test_rejects_missing_scenario(self):
        report = SUMMARY.analyze(self.write(self.valid_rows()[:-1]), 1)
        self.assertFalse(report['valid'])
        self.assertFalse(report['accepted'])

    def test_rejects_tampered_latency(self):
        rows = self.valid_rows()
        rows[0]['zero_command_latency_ms'] = 100.0
        report = SUMMARY.analyze(self.write(rows), 1)
        self.assertFalse(report['valid'])


if __name__ == '__main__':
    unittest.main()
