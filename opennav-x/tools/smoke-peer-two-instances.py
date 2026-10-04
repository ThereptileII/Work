#!/usr/bin/env python3
"""Concurrent peer containment using two NEW disconnected OpenCPN profiles.

Linux is a development check. Native Windows must run on disposable hosted CI.
This observes process-owned TCP listeners and supported application logs; it is
not a packet capture or a proof about every possible outbound network path.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from diagnostic_snapshot import read_json_snapshot
from peer_boundary import (PeerBoundary, _CERT_BYTES, _CLIENT_KEY, _KEY_BYTES,
                           _SERVER_KEY)


ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fixtures = load("profile-fixtures")


def native_temp_root():
    if os.name != "nt":
        return Path("/tmp")
    if (os.environ.get("GITHUB_ACTIONS", "").lower() != "true" or
            os.environ.get("RUNNER_ENVIRONMENT") != "github-hosted" or
            not os.environ.get("RUNNER_TEMP")):
        raise RuntimeError("Native Windows peer test requires disposable GitHub-hosted CI")
    result = Path(os.environ["RUNNER_TEMP"]).resolve(strict=True)
    if not result.is_dir():
        raise RuntimeError("RUNNER_TEMP is not a directory")
    return result


def wait_ready(process, profile):
    deadline = time.monotonic() + 75
    logfile = profile / "opencpn.log"
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"OpenCPN {process.pid} exited before startup: {process.returncode}")
        if logfile.exists() and "OnInitTimer...Finalize Canvases" in logfile.read_text(errors="replace"):
            try:
                return read_json_snapshot(profile / "opennav-diagnostics.json")
            except (FileNotFoundError, ValueError):
                pass
        time.sleep(.2)
    raise RuntimeError(f"OpenCPN {process.pid} did not finish startup")


def assert_no_secrets(profile, stdout, stderr):
    secrets = (_CLIENT_KEY.encode(), _SERVER_KEY.encode(),
               _CLIENT_KEY.split(":", 1)[1].rstrip(";").encode(),
               _SERVER_KEY.split(":", 1)[1].rstrip(";").encode(),
               _CERT_BYTES.strip(), _KEY_BYTES.strip())
    for path in (stdout, stderr, profile / "opencpn.log"):
        if not path.exists():
            raise RuntimeError(f"required application output missing: {path.name}")
        content = path.read_bytes()
        if any(secret in content for secret in secrets):
            raise AssertionError(f"seeded peer secret appeared in {path.name}")


def run(args):
    exe, build = args.exe.resolve(strict=True), args.build.resolve(strict=True)
    temp_parent = native_temp_root()
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    report = {"authority": "native Windows CI" if os.name == "nt" else "Linux development",
              "test_source_commit": subprocess.check_output(
                  ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "test_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "executable_sha256": hashlib.sha256(exe.read_bytes()).hexdigest(),
              "profiles": [], "observations": [], "result": "FAILED"}
    processes = []
    xserver = None
    with tempfile.TemporaryDirectory(prefix="opennav peer ", dir=temp_parent) as temporary:
        base = Path(temporary)
        env = os.environ.copy()
        if os.name != "nt":
            display_number = 98
            while Path(f"/tmp/.X{display_number}-lock").exists():
                display_number += 1
            env["DISPLAY"] = f":{display_number}"
            with (evidence / "peer-two-xvfb.stdout.log").open("wb") as out, \
                    (evidence / "peer-two-xvfb.stderr.log").open("wb") as err:
                xserver = subprocess.Popen(
                    ["Xvfb", env["DISPLAY"], "-screen", "0", "1280x800x24", "-nolisten", "tcp"],
                    env=env, stdout=out, stderr=err)
            time.sleep(1)
            if xserver.poll() is not None:
                raise RuntimeError("private X server exited before application launch")
        else:
            load("windows-ui").ensure_desktop()
        try:
            for index in (1, 2):
                profile = base / f"profile {index}"
                subprocess.run([sys.executable, str(ROOT / "tools/prepare-test-profile.py"),
                                "--build", str(build), "--profile", str(profile)],
                               check=True, capture_output=True)
                fixtures.seed(profile)
                boundary = PeerBoundary(profile)
                expected = fixtures.snapshot(profile)
                stdout = evidence / f"peer-two-{index}.stdout.log"
                stderr = evidence / f"peer-two-{index}.stderr.log"
                with stdout.open("wb") as out, stderr.open("wb") as err:
                    process = subprocess.Popen([str(exe), "--configdir", str(profile),
                                                "--no_opengl", "--xnav"],
                                               env=env, stdout=out, stderr=err)
                processes.append((process, profile, boundary, expected, stdout, stderr))
                report["profiles"].append({"index": index, "pid": process.pid,
                                           "profile": str(profile)})
            # Require both processes to be live after both independent profiles
            # have completed deferred initialization. Sequential mode tests do
            # not exercise this state.
            for process, profile, *_ in processes:
                diagnostic = wait_ready(process, profile)
                assert diagnostic.get("build_commit"), "running binary omitted its build commit"
                report["profiles"][len(report["observations"])]["build_commit"] = diagnostic.get("build_commit")
                report["observations"].append({"startup_pid": process.pid})
            assert all(process.poll() is None for process, *_ in processes), "two live apps required"
            for index, (process, profile, boundary, expected, stdout, stderr) in enumerate(processes):
                observed = boundary.observe(process.pid)
                assert fixtures.snapshot(profile) == expected, "navigation fixture changed while both apps ran"
                assert_no_secrets(profile, stdout, stderr)
                report["observations"][index].update(observed)
            for process, profile, *_ in processes:
                if os.name == "nt":
                    ui = load("windows-ui")
                    handle, observed_pid = ui.wait_window("SKAGER / OpenCPN", process.pid)
                    assert observed_pid == process.pid
                    ui.close(handle)
                else:
                    command = subprocess.run([str(exe), "--configdir", str(profile),
                                              "--remote", "--quit"], env=env,
                                             capture_output=True, timeout=20)
                    assert command.returncode == 0, "profile-scoped remote quit failed"
                assert process.wait(timeout=35) == 0, "application did not exit cleanly"
            for process, profile, boundary, expected, stdout, stderr in processes:
                boundary.assert_preserved()
                assert fixtures.snapshot(profile) == expected, "navigation fixture changed after close"
                assert_no_secrets(profile, stdout, stderr)
            report["result"] = "PASS"
        except BaseException as error:
            report["error"] = repr(error)
            raise
        finally:
            for process, *_ in processes:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=10)
            # Retain the application log for a failed gate, with fixture
            # credentials removed before it enters the CI evidence artifact.
            for index, (_, profile, _, _, _, _) in enumerate(processes, 1):
                cert, key = profile / "cert.pem", profile / "key.pem"
                report["profiles"][index - 1]["cert_preserved_at_exit"] = (
                    cert.is_file() and cert.read_bytes() == _CERT_BYTES)
                report["profiles"][index - 1]["key_preserved_at_exit"] = (
                    key.is_file() and key.read_bytes() == _KEY_BYTES)
                logfile = profile / "opencpn.log"
                if logfile.exists():
                    content = logfile.read_bytes()
                    for secret in (_CLIENT_KEY.encode(), _SERVER_KEY.encode(),
                                   _CLIENT_KEY.split(":", 1)[1].rstrip(";").encode(),
                                   _SERVER_KEY.split(":", 1)[1].rstrip(";").encode(),
                                   _CERT_BYTES.strip(), _KEY_BYTES.strip()):
                        content = content.replace(secret, b"[REDACTED]")
                    (evidence / f"peer-two-{index}.opencpn.log").write_bytes(content)
            if xserver is not None:
                xserver.terminate()
                xserver.wait(timeout=10)
            (evidence / "peer-two-instances-results.json").write_text(
                json.dumps(report, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    default_exe = ROOT / "build/xnav-install" / ("opencpn.exe" if os.name == "nt" else "bin/opencpn")
    parser.add_argument("--exe", type=Path, default=default_exe)
    parser.add_argument("--build", type=Path, default=ROOT / "build" /
                        ("xnav-windows" if os.name == "nt" else "xnav-linux"))
    parser.add_argument("--evidence", type=Path, default=ROOT / "evidence/local")
    run(parser.parse_args())
