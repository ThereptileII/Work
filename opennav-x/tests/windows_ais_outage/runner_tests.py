#!/usr/bin/env python3
"""Portable refusal/evidence checks only; these do not prove native WFP behavior."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

RUNNER = Path(__file__).resolve().parents[2] / "tools/test-windows-ais-outage.py"
spec = importlib.util.spec_from_file_location("outage_runner", RUNNER)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class Client:
    def __init__(self, events, alive=True):
        self.events = events
        self.process = self
        self.alive = alive

    def poll(self):
        return None if self.alive else 124

    def snapshot(self, event=None, since=0):
        return [e.copy() for e in self.events if e["received_monotonic"] >= since and
                (event is None or e["event"] == event)]


def traffic():
    return [{"event": "flow_error", "connection": 1, "tick": 1001, "received_monotonic": 1}] + [
        {"event": "connect_fail", "tick": 1000 + int(t * 1000), "received_monotonic": t}
        for t in (2 + i * .5 for i in range(245))]


class EvidenceTests(unittest.TestCase):
    def observe(self, events, end=124, alive=True):
        return runner.blocked_observation(Client(events, alive), 1, {"commit_before_tick": 1000},
                                          .5, 2, end, True)

    def test_complete_122_second_failed_retry_record(self):
        self.assertEqual(self.observe(traffic())["failed_retries_after_grace"], 245)

    def test_late_success_is_not_hidden_by_initial_packet_loss(self):
        for kind in ("echo", "connected"):
            with self.subTest(kind=kind), self.assertRaises(RuntimeError):
                self.observe(traffic() + [{"event": kind, "received_monotonic": 123, "tick": 123000}])

    def test_stopped_retrying_after_initial_failure_is_not_packet_loss(self):
        with self.assertRaisesRegex(RuntimeError, "keep attempting"):
            self.observe(traffic()[:10])

    def test_silence_with_dead_client_is_not_packet_loss(self):
        with self.assertRaisesRegex(RuntimeError, "remained alive"):
            self.observe(traffic(), alive=False)

    def test_short_window_cannot_cover_121_second_gate(self):
        with self.assertRaisesRegex(RuntimeError, "shorter"):
            self.observe(traffic(), end=123.9)

    def test_no_established_flow_failure_is_refused(self):
        with self.assertRaisesRegex(RuntimeError, "established flow"):
            self.observe(traffic()[1:])

    def test_precommit_failure_is_refused(self):
        events = traffic()
        events[0]["tick"] = 999
        with self.assertRaisesRegex(RuntimeError, "established flow"):
            self.observe(events)

    def test_deadman_and_forced_kill_are_distinct(self):
        runner.validate_exit("deadman-expiry", 124, 135, True)
        runner.validate_exit("forced-death", 1, 124, True)
        for name, code, elapsed in (("deadman-expiry", 1, 135), ("deadman-expiry", 0, 135),
                                    ("deadman-expiry", 124, 15), ("deadman-expiry", 124, 150),
                                    ("forced-death", 124, 135), ("forced-death", 0, 124),
                                    ("forced-death", 1, 132)):
            with self.subTest(name=name, code=code, elapsed=elapsed), self.assertRaises(RuntimeError):
                runner.validate_exit(name, code, elapsed, True)

    def test_native_clock_checks_gap_after_122_seconds_before_death(self):
        active = {"commit_after_tick": 2000}
        for kind in ("echo", "connected"):
            client = Client([{"event": kind, "tick": 130000, "received_monotonic": 140}])
            with self.subTest(kind=kind), self.assertRaisesRegex(RuntimeError, "before selected native"):
                runner.validate_native_block(client, active, 136000)
        runner.validate_native_block(Client([{"event": "echo", "tick": 136001,
                                              "received_monotonic": 140}]), active, 136000)

    def test_reader_failure_and_incomplete_reader_refuse_finalization(self):
        class Process:
            def poll(self): return 0
            def wait(self, timeout): return 0
        class Reader:
            def __init__(self, alive): self.alive = alive
            def join(self, timeout): pass
            def is_alive(self): return self.alive
        for alive, error in ((True, None), (False, "disk write failed")):
            process = runner.Process.__new__(runner.Process)
            process.process, process.thread, process.read_error = Process(), Reader(alive), error
            with self.subTest(alive=alive, error=error), self.assertRaises(RuntimeError):
                process.finish()

    @unittest.skipIf(sys.platform == "win32", "this fixture checks native-platform refusal")
    def test_linux_refusal_retains_failure_without_launching_runtime(self):
        with tempfile.TemporaryDirectory(prefix="ais-outage-refusal-") as directory:
            output = Path(directory) / "proof"
            env = dict(os.environ, RUNNER_TEMP=directory, CI="true", GITHUB_ACTIONS="true",
                       RUNNER_ENVIRONMENT="github-hosted", XNAV_DISPOSABLE_AIS_OUTAGE="1",
                       GITHUB_SHA="a" * 40)
            result = subprocess.run([sys.executable, str(RUNNER), "--output", str(output),
                                     "--confirm-disposable", "--long-deadman"],
                                    env=env, capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 1)
            report = json.loads((output / "result.json").read_text())
            self.assertFalse(report["passed"])
            self.assertEqual(report["error"], "native Windows required")
            self.assertFalse((output / "runtime").exists())


if __name__ == "__main__":
    unittest.main()
