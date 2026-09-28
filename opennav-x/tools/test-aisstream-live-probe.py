#!/usr/bin/env python3
"""CLI/privacy gates only; never requires the real AIS service or a real key."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--client", type=Path, required=True)
a = p.parse_args()
client = str(a.client.resolve())
checks = 0


def check(condition):
    global checks
    assert condition, f"live probe CLI/privacy check {checks + 1}"
    checks += 1


def run(*args):
    env = dict(os.environ)
    env.pop("AISSTREAM_API_KEY", None)
    result = subprocess.run([client, *args], capture_output=True, text=True,
                            env=env, timeout=15)
    check(len(result.stdout) + len(result.stderr) < 16384)
    return result


description = run("--describe")
check(description.returncode == 0)
data = json.loads(description.stdout)
check(data["endpoint"] == "wss://stream.aisstream.io/v0/stream")
check(data["maximum_observation_seconds"] == 45)
check(data["marine_equipment"] is False and data["profile_access"] is False)
check(data["secret_output"] is False)
marker = "a-user-might-accidentally-pass-a-key-here"
for args in [(), ("--read-only-live-ais",), ("stockholm",),
             ("--read-only-live-ais", marker), ("--describe", marker),
             ("--read-only-live-ais", "stockholm", marker)]:
    result = run(*args)
    check(result.returncode == 2)
    check(marker not in result.stdout + result.stderr)
if sys.platform != "win32":
    # The Linux production credential factory is environment-only. Removing
    # that one variable guarantees no credential and hence no socket attempt.
    result = run("--read-only-live-ais", "stockholm")
    check(result.returncode == 4)
    rows = [json.loads(line) for line in result.stdout.splitlines()]
    states = [r["state"] for r in rows if r["event"] == "connection"]
    check(states in [["credential_missing"], ["offline", "credential_missing"]])
    report = rows[-1]
    check(report["peak_target_count"] == 0 and report["accepted_reports"] == 0)
    check(report["subscription_confirmed"] is False)
    check(report["disabled_and_cleared"] is True)
print(f"PASS: {checks} offline live-probe CLI/privacy checks; no live service acceptance")
