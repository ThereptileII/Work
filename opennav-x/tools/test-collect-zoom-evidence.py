#!/usr/bin/env python3
"""Portable evidence-boundary tests; not native Windows qualification."""
import argparse
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('collector', Path(__file__).with_name('collect-zoom-evidence.py'))
collector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(collector)


class EvidenceTests(unittest.TestCase):
    def test_allowlist_and_unknowns(self):
        snapshot = {
            'build_commit': 'test-commit', 'publication_monotonic_ms': '1000',
            'secret': 'private', 'route': {'id': 'private', 'waypoint_count': 12},
            'runtime': {'chart': {'latitude': 60, 'longitude': 18,
                'scale_ppm': .001, 'quilt_members': [{'file': 'private.oesenc',
                'native_scale': 22000, 'type': 7}]},
                'online_ais': {'chart_marks': 3, 'targets': 8, 'token': 'private'}}}
        result = collector.observation(snapshot)
        self.assertNotIn('private', json.dumps(result))
        self.assertNotIn('latitude', json.dumps(result))
        self.assertEqual(result['counts']['online_ais']['chart_marks'], 3)
        self.assertEqual(result['counts']['active_route_waypoints'], 12)
        self.assertIsNone(result['counts']['local_ais_targets'])
        self.assertIsNone(result['chart']['viewport_scale_denominator'])
        self.assertIsNone(result['chart_frame_time_ms'])
        self.assertIsNone(collector.observation({})['chart']['quilt_members'])

    def test_stale_heartbeat_and_missing_timestamp(self):
        heartbeat = collector.Heartbeat()
        data = {'publication_monotonic_ms': '1000'}
        self.assertEqual(heartbeat.observe(data, 10)['status'], 'first_observation')
        self.assertEqual(heartbeat.observe(data, 12)['unchanged_ms'], 2000)
        self.assertIsNone(heartbeat.observe({}, 13)['unchanged_ms'])
        self.assertEqual(heartbeat.observe({'publication_monotonic_ms': '2000'}, 14),
                         {'status': 'new_publication', 'unchanged_ms': 0})

    def test_bounded_collection_preserves_gaps_and_identity_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            profile = root / 'profile'; profile.mkdir()
            diagnostic = profile / 'opennav-diagnostics.json'
            original = json.dumps({'build_commit': 'candidate', 'publication_monotonic_ms': '1'})
            diagnostic.write_text(original)
            exe = root / 'fixture.exe'; exe.write_bytes(b'fixture, not a real application')
            args = argparse.Namespace(pid=42, exe=exe, package_commit='candidate',
                disposable_profile=profile, diagnostics=diagnostic,
                output=root / 'output', seconds=5, confirm_disposable=True)
            class Clock:
                now = 0
                @classmethod
                def sleep(cls, seconds):
                    cls.now += seconds
                    if cls.now == .25:
                        diagnostic.unlink()  # atomic replacement gap
                    elif cls.now == .5:
                        diagnostic.write_text('{invalid')
                    elif cls.now == .75:
                        diagnostic.write_text('{"build_commit":"other"}')
                    elif cls.now == 1:
                        diagnostic.write_text(original)
            class Process:
                closed = False
                def __init__(self, pid, expected):
                    self.exe, self.created_filetime = expected, 123456
                def usage(self):
                    return self.created_filetime, Clock.now * 1.5, 1000000
                def close(self):
                    Process.closed = True
            fake_os = types.SimpleNamespace(name='nt', fstat=collector.os.fstat)
            with patch.object(collector, 'os', fake_os), patch.object(collector, 'WindowsProcess', Process), \
                 patch.object(collector.time, 'monotonic', lambda: Clock.now), \
                 patch.object(collector.time, 'sleep', Clock.sleep):
                collector.collect(args)
            rows = [json.loads(line) for line in (args.output / 'samples.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows), 20)
            self.assertEqual(rows[1]['diagnostic_error'], 'FileNotFoundError')
            self.assertEqual(rows[2]['diagnostic_error'], 'JSONDecodeError')
            self.assertEqual(rows[3]['diagnostic_error'], 'ValueError')
            self.assertEqual(rows[4]['cpu_percent_one_core'], 150)
            self.assertEqual(rows[-1]['publication_observation']['unchanged_ms'], 4750)
            self.assertTrue(Process.closed)
            self.assertEqual(diagnostic.read_text(), original)
            self.assertEqual(json.loads((args.output / 'completion.json').read_text())['acceptance'], 'not_assessed')
            self.assertEqual(json.loads((args.output / 'metadata.json').read_text())['exe_sha256'], collector.sha256(exe))

    def test_reject_nonfinite_evidence(self):
        with self.assertRaises(ValueError):
            json.loads('{"value": NaN}', parse_constant=collector.reject_nonfinite)


if __name__ == '__main__':
    unittest.main()
