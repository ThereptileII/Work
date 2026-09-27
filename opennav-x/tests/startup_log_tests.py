"""Readiness observes the new launch even when upstream rotates its log."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "startup_log", Path(__file__).resolve().parents[1] / "tools/startup-log.py")
log = importlib.util.module_from_spec(spec)
spec.loader.exec_module(log)

OLD = log.START + b"5.12.4 restarted yesterday\n" + log.READY + b"\n"
NEW = log.START + b"5.12.4 restarted today\n" + log.READY + b"\n"


class StartupReadiness(unittest.TestCase):
    def test_initial_and_appended_startup(self):
        self.assertTrue(log.initialized_since(b"", NEW))
        self.assertTrue(log.initialized_since(OLD, OLD + b"close normally\n" + NEW))

    def test_real_rotation_preserves_new_startup_proof(self):
        previous = OLD * 7 + b"large previous log" * 100000
        self.assertTrue(log.initialized_since(previous, NEW))
        self.assertTrue(log.initialized_since(previous, b"warning before startup\n" + NEW))

    def test_old_or_unfinished_markers_do_not_qualify(self):
        for before, current in ((OLD, OLD), (OLD, OLD + log.READY),
                                (OLD, b""), (OLD, log.READY),
                                (OLD, log.START + b"5.12.4\n"),
                                (OLD, OLD + NEW + log.START + b"unfinished"),
                                (OLD, OLD + b"unrelated finalization\n")):
            with self.subTest(current=current[-80:]):
                self.assertFalse(log.initialized_since(before, current))


if __name__ == "__main__":
    unittest.main()
