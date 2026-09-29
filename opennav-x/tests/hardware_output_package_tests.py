"""Product packaging must fail closed on missing or unqualified output policy."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from hardware_output_policy import require_status_only


class OutputPolicyTests(unittest.TestCase):
    def test_status_only(self):
        require_status_only({'xnav_hardware_output_policy': 'status-only'})

    def test_unqualified_or_missing(self):
        for value in (None, True, False, 0, 1, [], {}, 'STATUS-ONLY',
                      'status-only ', 'test-loopback-only', 'physical-output'):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    require_status_only({'xnav_hardware_output_policy': value})
        for report in ({}, None, [], 'status-only', True):
            with self.assertRaises(ValueError):
                require_status_only(report)


if __name__ == '__main__':
    unittest.main()
