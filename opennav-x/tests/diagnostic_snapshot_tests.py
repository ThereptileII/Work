import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from diagnostic_snapshot import read_json_snapshot


class DiagnosticSnapshotTests(unittest.TestCase):
    def test_native_unicode_control_labels_survive_locale_independent_read(self):
        # Native diagnostics publish UTF-8, even when Windows' default text
        # encoding is CP1252. The wrong decoding is valid JSON but changes the
        # control identity, so the actual product interaction cannot find it.
        labels = ["+1°", "−", "Arkösund"]
        report = {"runtime": {"display": {"product_controls": [
            {"label": label, "visible": True, "enabled": True,
             "width": 96, "height": 56} for label in labels]}}}
        encoded = json.dumps(report, ensure_ascii=False).encode("utf-8")
        wrong = json.loads(encoded.decode("cp1252"))
        self.assertNotEqual(wrong, report)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "opennav-diagnostics.json"
            path.write_bytes(encoded)
            self.assertEqual(read_json_snapshot(path), report)

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
