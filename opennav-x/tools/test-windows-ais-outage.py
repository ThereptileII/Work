#!/usr/bin/env python3
"""Disposable hosted-Windows WFP experiment. Never run against product/boat traffic."""
from __future__ import annotations

import argparse
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import socket
import subprocess
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "tests/windows_ais_outage"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def guards(env, current_platform, confirmed):
    require(confirmed, "--confirm-disposable is required")
    require(current_platform == "win32", "native Windows required")
    for key, expected in (("CI", "true"), ("GITHUB_ACTIONS", "true"),
                          ("RUNNER_ENVIRONMENT", "github-hosted"),
                          ("XNAV_DISPOSABLE_AIS_OUTAGE", "1")):
        require(env.get(key) == expected, f"refused: {key} is not {expected}")
    require(bool(env.get("RUNNER_TEMP")), "RUNNER_TEMP required")
    revision = env.get("GITHUB_SHA", "")
    require(len(revision) == 40 and all(c in "0123456789abcdef" for c in revision),
            "exact GITHUB_SHA required")


class Job:
    """Only the filter helper joins this kill-on-close job; never the product."""
    def __init__(self):
        class Limits(ctypes.Structure):
            _fields_ = [("process_time", ctypes.c_longlong), ("job_time", ctypes.c_longlong),
                        ("flags", wintypes.DWORD), ("min_ws", ctypes.c_size_t),
                        ("max_ws", ctypes.c_size_t), ("processes", wintypes.DWORD),
                        ("affinity", ctypes.c_size_t), ("priority", wintypes.DWORD),
                        ("scheduling", wintypes.DWORD)]

        class IO(ctypes.Structure):
            _fields_ = [(name, ctypes.c_ulonglong) for name in
                        ("reads", "writes", "other", "read_bytes", "write_bytes", "other_bytes")]

        class Extended(ctypes.Structure):
            _fields_ = [("basic", Limits), ("io", IO), ("process_memory", ctypes.c_size_t),
                        ("job_memory", ctypes.c_size_t), ("peak_process", ctypes.c_size_t),
                        ("peak_job", ctypes.c_size_t)]

        self.kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        self.kernel.CreateJobObjectW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR]
        self.kernel.CreateJobObjectW.restype = wintypes.HANDLE
        self.kernel.SetInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD]
        self.kernel.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
        self.kernel.CloseHandle.argtypes = [wintypes.HANDLE]
        self.handle = self.kernel.CreateJobObjectW(None, None)
        require(self.handle, "create helper-only job failed")
        info = Extended()
        info.basic.flags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
        if not self.kernel.SetInformationJobObject(self.handle, 9, ctypes.byref(info), ctypes.sizeof(info)):
            self.close()
            raise RuntimeError("set helper job lifetime failed")

    def assign(self, process):
        require(self.kernel.AssignProcessToJobObject(self.handle, int(process._handle)),
                "assign helper job failed; no arm command sent")

    def close(self):
        if self.handle:
            self.kernel.CloseHandle(self.handle)
            self.handle = None


class Process:
    def __init__(self, command, folder, name):
        self.events = []
        self.lock = threading.Lock()
        self.log = (folder / f"{name}.jsonl").open("w", encoding="utf-8")
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        for line in self.process.stdout:
            try:
                value = json.loads(line)
                require(isinstance(value, dict), "non-object native output")
            except (ValueError, RuntimeError):
                value = {"event": "diagnostic", "text": line.strip()[:1000]}
            value["received_monotonic"] = time.monotonic()
            with self.lock:
                self.events.append(value)
                if len(self.events) > 2000:
                    self.process.kill()
                    break
            self.log.write(json.dumps(value) + "\n")
            self.log.flush()
        self.log.close()

    def snapshot(self, event=None, since=0):
        with self.lock:
            return [e.copy() for e in self.events
                    if e["received_monotonic"] >= since and (event is None or e.get("event") == event)]

    def send(self, value):
        self.process.stdin.write(value + "\n")
        self.process.stdin.flush()

    def finish(self):
        if self.process.poll() is None:
            self.process.kill()  # retained process handle, never a discovered PID
        self.process.wait(timeout=3)
        self.thread.join(timeout=3)


class EchoServer:
    """Python stdlib server; accepts only literal IPv4/IPv6 loopback sockets."""
    def __init__(self):
        self.stop = threading.Event()
        self.listeners = []
        self.threads = []
        self.connections = []
        try:
            for family, address in ((socket.AF_INET, "127.0.0.1"), (socket.AF_INET6, "::1")):
                listener = socket.socket(family, socket.SOCK_STREAM)
                self.listeners.append(listener)
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
                if family == socket.AF_INET6:
                    listener.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
                listener.bind((address, 0 if family == socket.AF_INET else self.port))
                if family == socket.AF_INET:
                    self.port = listener.getsockname()[1]
                    require(49152 <= self.port <= 65535, "OS-selected port outside guarded ephemeral range")
                listener.listen(16)
                listener.settimeout(.2)
                thread = threading.Thread(target=self.accept, args=(listener,), daemon=True)
                self.threads.append(thread)
                thread.start()
        except Exception:
            self.close()
            raise

    def accept(self, listener):
        while not self.stop.is_set():
            try:
                connection, address = listener.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            if address[0] not in ("127.0.0.1", "::1"):
                connection.close()
                continue
            connection.settimeout(.3)
            self.connections.append(connection)
            thread = threading.Thread(target=self.echo, args=(connection,), daemon=True)
            self.threads.append(thread)
            thread.start()

    def echo(self, connection):
        try:
            while not self.stop.is_set():
                try:
                    data = connection.recv(8)
                    if not data:
                        break
                    connection.sendall(data)
                except socket.timeout:
                    continue
        except OSError:
            pass
        finally:
            connection.close()

    def close(self):
        self.stop.set()
        for connection in self.listeners + self.connections:
            connection.close()
        for thread in self.threads:
            thread.join(timeout=.5)


def wait_until(predicate, seconds, description):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(.025)
    raise RuntimeError("deadline: " + description)


def wait_event(process, event, seconds, description):
    def observe():
        value = next(iter(process.snapshot(event)), None)
        if value:
            return value
        code = process.process.poll()
        if code is not None:
            # Drain the finite pipe before reporting the original native error.
            process.thread.join(timeout=.2)
            value = next(iter(process.snapshot(event)), None)
            if value:
                return value
            diagnostics = process.snapshot("diagnostic")
            detail = diagnostics[-1].get("text", "") if diagnostics else "no native diagnostic"
            raise RuntimeError(f"helper exited {code} before {description}: {detail}")
        return None
    return wait_until(observe, seconds, description)


def audit(exe, keys, folder, phase):
    attempts = []
    end = time.monotonic() + 3
    while time.monotonic() < end:
        command = [str(exe), "--audit", keys["sublayer"], keys["filter4"], keys["filter6"]]
        result = subprocess.run(command, capture_output=True, text=True, timeout=3)
        attempts.append({"command": command, "exit": result.returncode,
                         "stdout": result.stdout[:5000], "stderr": result.stderr[:5000]})
        (folder / f"{phase}-cleanup.json").write_text(json.dumps(attempts, indent=2) + "\n")
        if result.returncode == 0:
            records = [json.loads(line) for line in result.stdout.splitlines() if line.startswith("{")]
            require(any(r.get("event") == "audit" and r.get("owned_objects_absent") is True for r in records),
                    "cleanup success missing exact absence result")
            return attempts
        time.sleep(.1)
    raise RuntimeError("owned WFP objects not proven absent after helper exit")


def phase(name, exe, clients, folder, marker_hash, port, report):
    item = {"name": name, "passed": False}
    report["phases"].append(item)
    old = {family: clients[f"marker{family}"].snapshot("echo")[-1]["connection"] for family in (4, 6)}
    start = time.monotonic()
    helper = None
    keys = None
    job = Job()
    try:
        command = [str(exe), str(port), str(clients["marker4"].process.pid),
                   str(clients["marker6"].process.pid), marker_hash]
        item["command"] = command
        helper = Process(command, folder, name + "-helper")
        job.assign(helper.process)
        keys = wait_event(helper, "prepared", 4, "guarded helper preparation")
        item["identity"] = keys
        for family in (4, 6):
            marker = clients[f"marker{family}"]
            require(marker.snapshot("echo")[-1]["connection"] == old[family] and
                    not marker.snapshot("flow_error", start), "target reconnected before filter arming")
        item["armed_monotonic"] = time.monotonic()
        helper.send("arm")
        active = wait_event(helper, "active", 3, "verified dual-family filter commit")
        item["active"] = active
        require(active.get("scope_verified") is True and active.get("families") == [4, 6],
                "missing exact dual-family scope readback")
        time.sleep(1)  # Record, but exclude, bounded queued-delivery/reauthorization grace.
        quiet_start = time.monotonic()
        time.sleep(2.5)
        item["quiet_window_seconds"] = time.monotonic() - quiet_start
        item["blocked"] = {}
        for family in (4, 6):
            marker = clients[f"marker{family}"]
            observation = {
                "echoes_after_grace": len(marker.snapshot("echo", quiet_start)),
                "connections_after_grace": len(marker.snapshot("connected", quiet_start)),
                "failed_retries_after_grace": len(marker.snapshot("connect_fail", quiet_start)),
                "established_flow_failed": any(e["connection"] == old[family] and
                    e["tick"] >= active["commit_before_tick"]
                    for e in marker.snapshot("flow_error", item["armed_monotonic"])),
                "process_alive": marker.process.poll() is None,
            }
            item["blocked"][str(family)] = observation
        item["control_during_block"] = {
            str(family): {"echoes": len(clients[f"control{family}"].snapshot("echo", quiet_start)),
                          "errors": len(clients[f"control{family}"].snapshot("flow_error", start)) +
                                    len(clients[f"control{family}"].snapshot("connect_fail", start))}
            for family in (4, 6)}
        # Keep both observations even if the first family bypasses WFP loopback filtering.
        for family, observed in item["blocked"].items():
            require(observed["process_alive"] and observed["established_flow_failed"],
                    "IPv" + family + " established flow was not interrupted while marker remained alive")
            require(observed["echoes_after_grace"] == 0 and observed["connections_after_grace"] == 0 and
                    observed["failed_retries_after_grace"] >= 2,
                    "IPv" + family + " quiet blocked-retry window failed; no broad fallback permitted")
        require(helper.process.poll() is None, "filter helper expired before selected cleanup path")
        if name == "normal":
            helper.send("stop")
            require(helper.process.wait(timeout=3) == 0, "normal helper close failed")
        else:
            helper.process.kill()  # exact owned handle; Windows TerminateProcess exit code 1
            require(helper.process.wait(timeout=3) == 1,
                    "forced-death phase ended normally or by watchdog instead")
        item["helper_exit"] = helper.process.returncode
        item["cleanup"] = audit(exe, keys, folder, name)
        recovery_start = time.monotonic()
        for family in (4, 6):
            wait_until(lambda family=family: any(e["connection"] > old[family]
                       for e in clients[f"marker{family}"].snapshot("echo", recovery_start)),
                       5, f"IPv{family} new-connection echo recovery")
        item["recovered"] = True
        end = time.monotonic()
        item["control"] = {}
        for family in (4, 6):
            control = clients[f"control{family}"]
            echoes = control.snapshot("echo", start)
            times = [start] + [e["received_monotonic"] for e in echoes] + [end]
            gap = max(b - a for a, b in zip(times, times[1:]))
            observation = {"echoes": len(echoes), "maximum_gap_seconds": gap,
                           "errors": len(control.snapshot("flow_error", start)) + len(control.snapshot("connect_fail", start)),
                           "connections": len({e["connection"] for e in echoes})}
            item["control"][str(family)] = observation
            require(control.process.poll() is None and observation["echoes"] >= 5 and gap < 2 and
                    observation["errors"] == 0 and observation["connections"] == 1,
                    f"IPv{family} independent control continuity failed")
    finally:
        # Close the job even when stdin/pipe/wait handling fails. It contains only
        # this helper and was assigned before any filter could be committed.
        job.close()
        try:
            if helper:
                helper.finish()
        finally:
            if keys and "cleanup" not in item:
                # Even a failed behavioral proof must independently establish rollback.
                item["failure_cleanup"] = audit(exe, keys, folder, name + "-failure")
    item["passed"] = True


def run(output, report):
    revision = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip()
    require(revision == os.environ["GITHUB_SHA"], "checkout does not match GITHUB_SHA")
    report.update(source_commit=revision, github_sha=os.environ["GITHUB_SHA"],
                  os=platform.platform(), windows_version=list(sys.getwindowsversion()),
                  admin=bool(ctypes.windll.shell32.IsUserAnAdmin()), phases=[])
    require(report["admin"], "already-elevated disposable runner required")
    bfe = subprocess.run(["sc.exe", "query", "BFE"], capture_output=True, text=True, timeout=5)
    report["bfe_query"] = {"exit": bfe.returncode, "stdout": bfe.stdout[:5000], "stderr": bfe.stderr[:2000]}
    require(bfe.returncode == 0, "BFE read-only query failed")
    compiler = shutil.which("cl.exe")
    require(compiler, "MSVC developer environment with cl.exe required")
    version = subprocess.run([compiler], capture_output=True, text=True, timeout=5)
    report["compiler"] = {"path": compiler, "version": (version.stdout + version.stderr)[:5000]}
    sources = [SOURCE / name for name in ("guard.h", "marker.cpp", "filter.cpp")] + [Path(__file__).resolve()]
    report["source_sha256"] = {str(p.relative_to(ROOT)): sha(p) for p in sources}
    folder = Path(os.environ["RUNNER_TEMP"]).resolve() / ("xnav-ais-outage-" + uuid.uuid4().hex)
    folder.mkdir()
    report["runtime_directory"] = str(folder)
    clients = {}
    server = None
    try:
        report["build_commands"] = []
        for role, source in (("marker", "marker.cpp"), ("control", "marker.cpp"), ("filter", "filter.cpp")):
            exe = folder / f"xnav-ais-outage-{role}.exe"
            command = [compiler, "/nologo", "/std:c++17", "/EHsc", "/W4", "/O2", "/MT", "/DUNICODE", "/D_UNICODE",
                       "/D_WIN32_WINNT=0x0602", "/Fo" + str(folder / f"{role}.obj"), "/Fe" + str(exe)]
            if role == "control":
                command.append("/DOUTAGE_CONTROL=1")
            command += [str(SOURCE / source), "/link", "ws2_32.lib"]
            if role == "filter":
                command += ["fwpuclnt.lib", "rpcrt4.lib", "bcrypt.lib", "advapi32.lib"]
            report["build_commands"].append(command)
            with (folder / f"{role}-compile.log").open("w") as log:
                subprocess.run(command, cwd=folder, stdout=log, stderr=log, timeout=60, check=True)
        report["binary_sha256"] = {p.name: sha(p) for p in folder.glob("*.exe")}
        server = EchoServer()
        report["server"] = {"addresses": ["127.0.0.1", "::1"], "port": server.port, "ipv6_only": True}
        for role in ("marker", "control"):
            for family in (4, 6):
                name = f"{role}{family}"
                clients[name] = Process([str(folder / f"xnav-ais-outage-{role}.exe"), str(family), str(server.port)], folder, name)
        wait_until(lambda: all(len(client.snapshot("echo")) >= 6 for client in clients.values()),
                   5, "all four actual established echo flows")
        report["baseline"] = {name: {"pid": client.process.pid, "echoes": len(client.snapshot("echo")),
                                     "connection": client.snapshot("echo")[-1]["connection"]}
                              for name, client in clients.items()}
        marker_hash = report["binary_sha256"]["xnav-ais-outage-marker.exe"]
        for name in ("normal", "forced-death"):
            phase(name, folder / "xnav-ais-outage-filter.exe", clients, folder, marker_hash, server.port, report)
    finally:
        cleanup_errors = []
        for name, client in clients.items():
            try:
                client.finish()
            except Exception as error:
                cleanup_errors.append(f"{name}: {error}")
        if server:
            try:
                server.close()
            except Exception as error:
                cleanup_errors.append(f"echo server: {error}")
        # Preserve failed compile/behavior evidence too; never delete the runtime directory.
        try:
            shutil.copytree(folder, output / "runtime", dirs_exist_ok=False)
        except Exception as error:
            cleanup_errors.append(f"evidence copy: {error}")
        report["finalization_errors"] = cleanup_errors
        require(not cleanup_errors, "process cleanup or evidence retention failed")
    report["passed"] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--confirm-disposable", action="store_true")
    args = parser.parse_args()
    report = {"passed": False, "scope": "disposable-loopback-WFP-only", "boat_qualified": False,
              "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    output = None
    try:
        # An invalid platform/opt-in can still retain a report at a fresh, safe output path.
        temporary = Path(os.environ.get("RUNNER_TEMP", "")).resolve()
        candidate = args.output.resolve()
        require(bool(os.environ.get("RUNNER_TEMP")) and temporary in candidate.parents,
                "output must be a fresh directory below RUNNER_TEMP")
        require(not candidate.exists(), "output already exists; refusing overwrite")
        candidate.mkdir(parents=True)
        output = candidate
        guards(os.environ, sys.platform, args.confirm_disposable)
        run(output, report)
    except Exception as error:
        report["passed"] = False
        report["error"] = str(error)[:2000]
    finally:
        report["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        if output:
            (output / "result.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
