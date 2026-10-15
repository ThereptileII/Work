#!/usr/bin/env python3
"""Portable regression for positive normal-startup Windows plugin discovery."""
import copy
import unittest

from bundled_plugins import require_bundled_windows_plugins

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


if __name__ == '__main__':
    unittest.main()
