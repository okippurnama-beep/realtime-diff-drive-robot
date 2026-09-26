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

"""Unit tests for the offline Nav2 benchmark summarizer."""

import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

import yaml


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'summarize_nav_benchmark.py'
)
SPEC = importlib.util.spec_from_file_location(
    'summarize_nav_benchmark',
    SCRIPT_PATH,
)
SUMMARY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUMMARY)


class NavBenchmarkSummaryTest(unittest.TestCase):
    """Exercise complete, incomplete, and inconsistent result sets."""

    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        directory = Path(self.temporary_directory.name)
        self.layout_path = directory / 'layout.yaml'
        self.csv_path = directory / 'results.csv'
        self.layout = {
            'version': 1,
            'robot': {'start': [0.3, 0.0, 0.0]},
            'goals': [
                {'name': 'goal_a', 'pose': [1.0, 0.0, 0.0]},
                {'name': 'goal_b', 'pose': [0.0, 1.0, 1.5708]},
            ],
        }
        self.layout_path.write_text(
            yaml.safe_dump(self.layout, sort_keys=False),
            encoding='utf-8',
        )

    def tearDown(self):
        self.temporary_directory.cleanup()

    def make_row(self, index, goal):
        """Create a consistent successful trial row."""
        target_x, target_y, target_yaw = goal['pose']
        return {
            'schema_version': 1,
            'run_id': '20260926T120000Z',
            'repetition': 1,
            'trial_index': index,
            'goal_name': goal['name'],
            'target_x_m': target_x,
            'target_y_m': target_y,
            'target_yaw_rad': target_yaw,
            'start_x_m': 0.3,
            'start_y_m': 0.0,
            'start_yaw_rad': 0.0,
            'finish_x_m': target_x - 0.04,
            'finish_y_m': target_y,
            'finish_yaw_rad': target_yaw - 0.02,
            'position_error_m': 0.04,
            'yaw_error_rad': 0.02,
            'action_status': 'SUCCEEDED',
            'status_code': 4,
            'nav2_error_code': 0,
            'nav2_error_msg': '',
            'navigation_time_s': 12.0 + index,
            'wall_time_s': 12.2 + index,
            'max_recoveries': 0,
            'final_distance_remaining_m': 0.08,
            'timed_out': False,
            'return_status': 'SUCCEEDED',
        }

    def write_rows(self, rows):
        """Write rows using the production schema order."""
        with self.csv_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=SUMMARY.REQUIRED_FIELDS,
            )
            writer.writeheader()
            writer.writerows(rows)

    def analyze(self):
        """Analyze the current test CSV."""
        return SUMMARY.analyze_results(
            [self.csv_path],
            self.layout_path,
        )

    def test_complete_run_is_valid_and_summarized(self):
        rows = [
            self.make_row(index, goal)
            for index, goal in enumerate(self.layout['goals'], start=1)
        ]
        self.write_rows(rows)

        report = self.analyze()

        self.assertTrue(report['valid'])
        self.assertEqual(report['overall']['trials'], 2)
        self.assertEqual(report['overall']['successes'], 2)
        self.assertEqual(
            report['overall']['success_rate_percent'],
            100.0,
        )
        self.assertIn('| `goal_a` |', SUMMARY.render_markdown(report))

    def test_missing_goal_makes_run_invalid(self):
        self.write_rows([self.make_row(1, self.layout['goals'][0])])

        report = self.analyze()

        self.assertFalse(report['valid'])
        self.assertTrue(any(
            'trial_index sequence' in message
            for message in report['validation']['errors']
        ))

    def test_target_mismatch_is_rejected(self):
        rows = [
            self.make_row(index, goal)
            for index, goal in enumerate(self.layout['goals'], start=1)
        ]
        rows[0]['target_x_m'] = 9.0
        self.write_rows(rows)

        report = self.analyze()

        self.assertFalse(report['valid'])
        self.assertTrue(any(
            'target x does not match layout' in message
            for message in report['validation']['errors']
        ))

    def test_return_failure_is_counted(self):
        rows = [
            self.make_row(index, goal)
            for index, goal in enumerate(self.layout['goals'], start=1)
        ]
        rows[1]['return_status'] = 'ABORTED'
        self.write_rows(rows)

        report = self.analyze()

        self.assertFalse(report['valid'])
        self.assertEqual(report['overall']['successes'], 1)
        self.assertEqual(
            report['overall']['failure_counts'],
            {'RETURN_ABORTED': 1},
        )


if __name__ == '__main__':
    unittest.main()
