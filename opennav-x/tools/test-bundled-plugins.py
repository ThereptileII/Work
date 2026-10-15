#!/usr/bin/env python3
"""Portable regression for positive normal-startup Windows plugin discovery."""
import copy
import unittest
from unittest.mock import Mock, patch

from bundled_plugins import require_bundled_windows_plugins, require_fresh_bundled_windows_plugins

# Extracted runtime.plugins from the first native phase of the retained public
# docs/evidence/native-visual-bccdbb1/charts-results.json. Only plugin fields;
# historical fixture shape, not acceptance evidence for the current candidate.
PLUGIN_RECORDS = [
    dict(name='ChartDownloader', api=113, version='1.4', initialized=False,
         file='chartdldr_pi.dll', enabled=False),
    dict(name='Dashboard', api=108, version='1.2', initialized=True,
         file='dashboard_pi.dll', enabled=True),
    dict(name='GRIB', api=116, version='5.0', initialized=True,
         file='grib_pi.dll', enabled=True),
    dict(name='WMM', api=108, version='1.1', initialized=True,
         file='wmm_pi.dll', enabled=True),
]


class BundledPlugins(unittest.TestCase):
    def check(self, records):
        require_bundled_windows_plugins({'runtime': {'plugins': records}})

    def test_extracted_positive_shape_accepts_disabled_chart_downloader(self):
        before = copy.deepcopy(PLUGIN_RECORDS)
        self.check(PLUGIN_RECORDS)
        self.assertEqual(PLUGIN_RECORDS, before)

    def test_each_missing_plugin_refuses(self):
        for index in range(4):
            with self.subTest(index=index), self.assertRaises(AssertionError):
                self.check(PLUGIN_RECORDS[:index] + PLUGIN_RECORDS[index+1:])

    def test_each_duplicate_including_windows_case_alias_refuses(self):
        for record in PLUGIN_RECORDS:
            for filename in (record['file'], record['file'].upper()):
                with self.subTest(filename=filename), self.assertRaises(AssertionError):
                    self.check(PLUGIN_RECORDS + [dict(record, file=filename)])

    def test_common_name_cannot_substitute_a_wrong_dll(self):
        for index in range(4):
            for filename in ('replacement.dll', 'prefix-' + PLUGIN_RECORDS[index]['file'],
                             'plugins/' + PLUGIN_RECORDS[index]['file']):
                records = copy.deepcopy(PLUGIN_RECORDS)
                records[index]['file'] = filename
                with self.subTest(index=index, filename=filename), self.assertRaises(AssertionError):
                    self.check(records)

    def test_empty_original_failure_and_malformed_records_refuse(self):
        for records in ([], None, [None], [{'name': 'Dashboard'}]):
            with self.subTest(records=records), self.assertRaises(AssertionError):
                self.check(records)

    def test_unrelated_plugin_does_not_break_exact_bundled_matches(self):
        self.check(PLUGIN_RECORDS + [dict(file='another_pi.dll', name='Dashboard')])


class FreshPublication(unittest.TestCase):
    def snapshot(self, epoch, records=PLUGIN_RECORDS):
        return dict(publication_monotonic_ms=epoch, publication_clock='live monotonic clock',
                    runtime={'plugins': copy.deepcopy(records)})

    def test_stale_pre_scan_snapshot_waits_only_for_new_publication(self):
        baseline = self.snapshot('533675', [])
        fresh = self.snapshot('534675')
        read = Mock(side_effect=[baseline, fresh])
        with patch('bundled_plugins.time.sleep') as sleep:
            self.assertIs(require_fresh_bundled_windows_plugins(read, baseline), fresh)
        self.assertEqual(read.call_count, 2)
        sleep.assert_called_once_with(.1)

    def test_first_newer_publication_missing_plugin_fails_without_retry(self):
        for records in ([], PLUGIN_RECORDS[:-1], PLUGIN_RECORDS + [PLUGIN_RECORDS[0]],
                        [dict(record, file='wrong.dll') for record in PLUGIN_RECORDS]):
            read = Mock(side_effect=[self.snapshot('101', records), self.snapshot('102')])
            with self.subTest(records=records), self.assertRaises(AssertionError):
                require_fresh_bundled_windows_plugins(read, self.snapshot('100'))
            self.assertEqual(read.call_count, 1)

    def test_previous_process_snapshot_is_not_a_restart_publication(self):
        previous = self.snapshot('99')
        baseline = self.snapshot('100')
        fresh = self.snapshot('101')
        read = Mock(side_effect=[previous, baseline, fresh])
        with patch('bundled_plugins.time.sleep'):
            self.assertIs(require_fresh_bundled_windows_plugins(read, baseline), fresh)
        self.assertEqual(read.call_count, 3)

    def test_unchanged_publication_times_out_even_with_all_plugins(self):
        baseline = self.snapshot('100')
        read = Mock(return_value=baseline)
        with patch('bundled_plugins.time.monotonic', side_effect=[0, 0, 8]), \
                patch('bundled_plugins.time.sleep'), self.assertRaisesRegex(AssertionError, 'did not arrive'):
            require_fresh_bundled_windows_plugins(read, baseline)
        read.assert_called_once_with()

    def test_missing_malformed_or_wrong_clock_epoch_refuses(self):
        invalid = [None, 100, True, '-1', '1.0', 'NaN', '１', '9' * 20,
                   '9223372036854775808']
        for epoch in invalid:
            with self.subTest(epoch=epoch), self.assertRaisesRegex(AssertionError, 'epoch'):
                require_fresh_bundled_windows_plugins(Mock(), self.snapshot(epoch))
        wrong_clock = self.snapshot('101');wrong_clock['publication_clock'] = 'recorded session clock'
        read = Mock(return_value=wrong_clock)
        with self.assertRaisesRegex(AssertionError, 'epoch'):
            require_fresh_bundled_windows_plugins(read, self.snapshot('100'))
        read.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
