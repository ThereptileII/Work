"""Meaningful denial matrix for old/mismatched package restart capabilities."""
import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from restart_capability import verified_restart_protocol


class CapabilityTests(unittest.TestCase):
    def setUp(self):
        self.app = {"contract": "OpenNavX.LoaderSelfTest.1", "passed": True,
                    "profile_initialized": False, "plugins_loaded": False,
                    "commissioning_restart_protocol": 1}
        self.helper = {"contract": "OpenNavX.RestartCapability.1", "role": "restart-helper",
                       "commissioning_restart_protocol": 1,
                       "profile_accessed": False, "child_started": False}

    def test_executed_matching_capability(self):
        self.assertEqual(verified_restart_protocol(self.app, self.helper), 1)

    def test_old_missing_or_invalid_capability(self):
        for source in ("app", "helper"):
            for value in (None, 0, 2, "1", True, 1.0):
                with self.subTest(source=source, value=value):
                    app, helper = copy.deepcopy((self.app, self.helper))
                    report = app if source == "app" else helper
                    if value is None:
                        report.pop("commissioning_restart_protocol")
                    else:
                        report["commissioning_restart_protocol"] = value
                    with self.assertRaises(ValueError):
                        verified_restart_protocol(app, helper)

    def test_unverified_loader_or_helper(self):
        mutations = [("app", "contract", "unknown"), ("app", "passed", False),
                     ("app", "profile_initialized", True), ("app", "plugins_loaded", True),
                     ("helper", "contract", "unknown"), ("helper", "role", "application"),
                     ("helper", "profile_accessed", True), ("helper", "child_started", True),
                     ("helper", "extra", "unexpected")]
        for source, key, value in mutations:
            with self.subTest(source=source, key=key):
                app, helper = copy.deepcopy((self.app, self.helper))
                (app if source == "app" else helper)[key] = value
                with self.assertRaises(ValueError):
                    verified_restart_protocol(app, helper)

    def test_malformed_report(self):
        for malformed in (None, [], "1", True):
            with self.assertRaises(ValueError):
                verified_restart_protocol(malformed, self.helper)
            with self.assertRaises(ValueError):
                verified_restart_protocol(self.app, malformed)


if __name__ == "__main__":
    unittest.main()
