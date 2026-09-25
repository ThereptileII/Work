import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from diagnostic_snapshot import read_json_snapshot


class DiagnosticSnapshotTests(unittest.TestCase):
    def test_access_race_returns_complete_failed_report_without_reinterpreting_it(self):
        path = Mock()
        path.read_text.side_effect = [PermissionError(), FileNotFoundError(),
                                      '{"result":"failed","check":"arrival"}']
        with patch("diagnostic_snapshot.time.sleep"):
            result = read_json_snapshot(path)
        self.assertEqual(result, {"result": "failed", "check": "arrival"})
        self.assertEqual(path.read_text.call_count, 3)

    def test_missing_or_locked_report_has_a_deadline(self):
        for error in (FileNotFoundError, PermissionError):
            with self.subTest(error=error):
                path = Mock()
                path.read_text.side_effect = error
                with patch("diagnostic_snapshot.time.monotonic", side_effect=[0, 0, 2]), \
                     patch("diagnostic_snapshot.time.sleep") as pause:
                    with self.assertRaises(error):
                        read_json_snapshot(path)
                self.assertEqual(path.read_text.call_count, 2)
                pause.assert_called_once_with(.02)

    def test_corrupt_json_is_not_retried(self):
        path = Mock()
        path.read_text.return_value = '{"result":'
        with self.assertRaises(json.JSONDecodeError):
            read_json_snapshot(path)
        path.read_text.assert_called_once_with(encoding="utf-8")

    def test_unrelated_io_error_is_not_retried(self):
        path = Mock()
        path.read_text.side_effect = OSError("device failure")
        with self.assertRaisesRegex(OSError, "device failure"):
            read_json_snapshot(path)
        self.assertEqual(path.read_text.call_count, 1)


if __name__ == "__main__":
    unittest.main()
