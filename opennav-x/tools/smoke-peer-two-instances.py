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
import re
import subprocess
import sys
import tempfile
import time
import zipfile

import staging_build_inputs as staging_inputs

from diagnostic_snapshot import read_json_snapshot
from smoke_startup import defer_boat_setup, native_setup_window
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
            except FileNotFoundError:
                pass
        time.sleep(.2)
    raise RuntimeError(f"OpenCPN {process.pid} did not finish startup")


SECRETS = (_CLIENT_KEY.encode(), _SERVER_KEY.encode(),
               _CLIENT_KEY.split(":", 1)[1].rstrip(";").encode(),
               _SERVER_KEY.split(":", 1)[1].rstrip(";").encode(),
               _CERT_BYTES.strip(), _KEY_BYTES.strip())


def redact(content):
    for secret in SECRETS:
        content = content.replace(secret, b"[REDACTED]")
    return content


def windows_startup_failure(process, profile, ui):
    """Read owned windows and bounded startup state before terminating a failure.

    No window activation, messages, input, profile writes or retry. Log output
    can remain buffered until normal exit, so a zero-byte log is not a stack.
    """
    clean = lambda value: redact(value.encode('utf-8')).decode('utf-8')
    state = {'pid': process.pid, 'exit_code_before_cleanup': process.poll(),
             'files': {}, 'windows': []}
    for name in ('opencpn.log', 'opennav-diagnostics.json', 'opennav-startup.state', 'startcheck.dat'):
        try:
            state['files'][name] = {'bytes': (profile / name).stat().st_size}
        except OSError as error:
            state['files'][name] = {'error': type(error).__name__}
    try:
        path = profile / 'opennav-diagnostics.json'
        with path.open('rb') as stream:
            raw = stream.read(1024 * 1024 + 1)
        if len(raw) > 1024 * 1024:
            raise ValueError('Diagnostic exceeds failure observation bound')
        data = json.loads(raw)
        scalar = lambda value: clean(value) if isinstance(value, str) else value if value is None or type(value) in (int, float) else 'invalid scalar'
        state['diagnostic'] = {
            'build_commit': scalar(data.get('build_commit')),
            'publication_monotonic_ms': scalar(data.get('publication_monotonic_ms')),
            'ui_ticks': scalar(data.get('runtime', {}).get('ui_update', {}).get('ticks'))}
    except (OSError, ValueError, TypeError, AttributeError) as error:
        state['diagnostic_error'] = type(error).__name__
    for handle, owner, title in ui.windows(process.pid)[:16]:
        if owner != process.pid:
            raise ValueError('Failure window observation changed owner')
        native_class = ui.C.create_unicode_buffer(128)
        ui.GetClassNameW(handle, native_class, len(native_class))
        window = {'title': clean(title), 'class': native_class.value,
                  'enabled': bool(ui.IsWindowEnabled(handle)), 'controls': []}
        for child, caption in ui.children(handle)[:64]:
            child_class = ui.C.create_unicode_buffer(128)
            ui.GetClassNameW(child, child_class, len(child_class))
            # Read only cached static/button captions, never editable fields.
            if child_class.value.casefold() in ('static', 'button'):
                window['controls'].append({'class': child_class.value,
                    'caption': clean(caption), 'enabled': bool(ui.IsWindowEnabled(child))})
        state['windows'].append(window)
    return state


def require_build(diagnostic, expected):
    if diagnostic.get("build_commit") != expected:
        raise AssertionError("running binary does not match the exact product commit")


def product_identity(exe, build, requested, receipt_path, harness, root=ROOT):
    """Consume the prior authenticated restore; never treat a caller SHA as authority."""
    expected = requested if requested is not None else harness
    require = staging_inputs.require
    require(isinstance(expected, str) and re.fullmatch('[0-9a-f]{40}', expected),
            'Exact product commit required')
    if receipt_path is None:
        require(expected == harness, 'A different product commit requires a restored input receipt')
        return expected, None
    root = root.resolve()
    receipt_file = staging_inputs.plain_file(root, 'evidence/local/staging-inputs.json')
    require(receipt_path.absolute() == receipt_file, 'Only the authenticated staging restore receipt may be used')
    raw = receipt_file.read_bytes()
    receipt = staging_inputs.strict_json(raw)
    require(isinstance(receipt, dict), 'Invalid staging restore receipt')
    identity = receipt.get('producer', {})
    require(isinstance(identity, dict), 'Invalid staging restore producer')
    require(identity == staging_inputs.producer(identity.get('commit'), identity.get('runId'),
                                                identity.get('runAttempt'), identity.get('job')) and
            identity['commit'] == expected and type(receipt.get('schema')) is int and
            receipt['schema'] == 1 and receipt.get('kind') == staging_inputs.KIND and
            receipt.get('status') == 'restored' and receipt.get('qualification') == 'not-run' and
            receipt.get('workspaceRoot') == str(root) and receipt.get('harnessCommit') == harness,
            'Peer restore receipt producer, harness or workspace differs')
    for field in ('archiveSha256', 'manifestSha256'):
        require(isinstance(receipt.get(field), str) and
                re.fullmatch('[0-9a-f]{64}', receipt[field]), 'Invalid restore digest')
    executable_name = 'build/xnav-install/opencpn.exe'
    executable = staging_inputs.plain_file(root, executable_name)
    require(exe == executable and build == root / 'build/xnav-windows',
            'Peer test requires the fixed restored application and build paths')
    # The restore already authenticated the archive against Actions metadata.
    # Its receipt binds this manifest, which in turn binds the executable bytes.
    archive = staging_inputs.plain_file(root, 'build/retained-staging-download/STAGING_BUILD_INPUTS.zip')
    with zipfile.ZipFile(archive) as source:
        entries = [entry for entry in source.infolist() if entry.filename == staging_inputs.MANIFEST]
        require(len(entries) == 1 and entries[0].file_size <= staging_inputs.MAX_MANIFEST,
                'Missing or ambiguous retained input manifest')
        encoded = source.read(entries[0])
    require(hashlib.sha256(encoded).hexdigest() == receipt['manifestSha256'],
            'Retained input manifest differs from authenticated restore')
    manifest = staging_inputs.strict_json(encoded)
    require(manifest.get('producer') == identity, 'Retained manifest producer differs')
    records = [record for record in manifest['files'] if record['path'] == executable_name]
    require(len(records) == 1 and type(records[0]['size']) is int and
            records[0]['size'] == executable.stat().st_size and
            records[0]['sha256'] == staging_inputs.sha(executable),
            'Peer executable differs from authenticated restore')
    return expected, hashlib.sha256(raw).hexdigest()


def assert_no_secrets(profile, stdout, stderr):
    for path in (stdout, stderr, profile / "opencpn.log"):
        if not path.exists():
            raise RuntimeError(f"required application output missing: {path.name}")
        content = path.read_bytes()
        if any(secret in content for secret in SECRETS):
            raise AssertionError(f"seeded peer secret appeared in {path.name}")


def defer_setup(process, profile, env):
    read = lambda: read_json_snapshot(profile / "opennav-diagnostics.json")
    if os.name == "nt":
        ui = load("windows-ui")
        return defer_boat_setup(
            read, lambda target: ui.pointer_text(process.pid, "Later"),
            native_window=lambda: native_setup_window(ui, process.pid))

    def click(target):
        dialogs = subprocess.check_output(
            ["xdotool", "search", "--all", "--onlyvisible", "--pid",
             str(process.pid), "--name", "^Boat Setup & Sensor Check$"],
            env=env, text=True).splitlines()
        if len(dialogs) != 1:
            raise AssertionError("expected one process-owned initial setup sheet")
        subprocess.run(
            ["xdotool", "windowraise", dialogs[0], "mousemove",
             str(target["x"] + target["width"] // 2),
             str(target["y"] + target["height"] // 2), "click", "1"],
            env=env, check=True)
    return defer_boat_setup(read, click)


def launch_arguments(exe, profile, *, portable=False):
    # wxMSW ignores wxSingleInstanceChecker.Create's directory argument. A
    # second ordinary --configdir process would compete for the first one's
    # session-wide mutex rather than test a second disconnected profile.
    return [str(exe), *(["--portable"] if portable else []),
            "--configdir", str(profile), "--no_opengl", "--xnav"]


def launch_plan(platform):
    return ((1, False), (2, platform == "nt"))


def launch_working_directory(profile, *, portable=False):
    # Upstream portable NormalizePath stores bundled resources relative to
    # PrivateDataDir. With an external --configdir, that is this profile, so
    # the process must start there for its tide/chart paths to resolve.
    return str(profile) if portable else None


def finalize_evidence(processes, xserver, outputs, evidence, report):
    """Publish only sanitized copies; independent failures never skip a stream.

    Raw child streams stay in the disposable profile root, outside the artifact
    directory. Even a failed stop/read/write cannot leave raw evidence behind.
    Return failures so the caller can preserve an already-active gate exception.
    """
    failures = []

    def attempt(stage, action):
        try:
            action()
        except BaseException as error:
            # Exception messages can themselves contain captured credentials.
            failures.append({'stage': stage, 'error': type(error).__name__})

    def stop(process):
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=10)

    for index, (process, *_) in enumerate(processes, 1):
        attempt(f'profile-{index}-cleanup', lambda: stop(process))
    if xserver is not None:
        attempt('xserver-cleanup', lambda: stop(xserver))
    for index, (_, profile, *_) in enumerate(processes, 1):
        for name, expected in (('cert', _CERT_BYTES), ('key', _KEY_BYTES)):
            def inspect():
                path = profile / (name + '.pem')
                report['profiles'][index - 1][name + '_preserved_at_exit'] = (
                    path.is_file() and path.read_bytes() == expected)
            attempt(f'profile-{index}-{name}', inspect)
    for source, destination in outputs:
        # OpenCPN may exit before opening its optional application log. The
        # launched child streams are mandatory; a log that was never written
        # has no bytes to retain or redact.
        if source.name == 'opencpn.log' and not source.exists():
            continue
        attempt(destination.name, lambda: destination.write_bytes(redact(source.read_bytes())))
    if failures:
        report['result'] = 'FAILED'
        report['finalization_errors'] = failures

    def sanitized(value):
        # Redact before JSON encoding, which escapes multiline PEM sentinels.
        if isinstance(value, str):
            return redact(value.encode('utf-8')).decode('utf-8')
        if isinstance(value, dict):
            return {key: sanitized(item) for key, item in value.items()}
        if isinstance(value, list):
            return [sanitized(item) for item in value]
        return value

    attempt('results-json', lambda: (evidence / 'peer-two-instances-results.json').write_text(
        json.dumps(sanitized(report), indent=2) + '\n', encoding='utf-8'))
    if failures:
        report['result'] = 'FAILED'
        report['finalization_errors'] = failures
    return failures


def run(args):
    exe, build = args.exe.resolve(strict=True), args.build.resolve(strict=True)
    harness = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    expected, receipt_sha = product_identity(
        exe, build, args.expected_commit, args.compiled_input_receipt, harness)
    temp_parent = native_temp_root()
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=True)
    report = {"authority": "native Windows CI" if os.name == "nt" else "Linux development",
              "test_source_commit": harness, "product_commit": expected,
              "restore_receipt_sha256": receipt_sha,
              "test_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "executable_sha256": hashlib.sha256(exe.read_bytes()).hexdigest(),
              "profiles": [], "observations": [], "result": "FAILED"}
    processes = []
    outputs = []
    xserver = None
    with tempfile.TemporaryDirectory(prefix="opennav peer ", dir=temp_parent,
                                     ignore_cleanup_errors=True) as temporary:
        base = Path(temporary)
        env = os.environ.copy()
        try:
            if os.name != "nt":
                display_number = 98
                while Path(f"/tmp/.X{display_number}-lock").exists():
                    display_number += 1
                env["DISPLAY"] = f":{display_number}"
                stdout, stderr = (base / f"peer-two-xvfb.{stream}.log"
                                  for stream in ('stdout', 'stderr'))
                outputs.extend((path, evidence / path.name) for path in (stdout, stderr))
                with stdout.open("wb") as out, stderr.open("wb") as err:
                    xserver = subprocess.Popen(
                        ["Xvfb", env["DISPLAY"], "-screen", "0", "1280x800x24", "-nolisten", "tcp"],
                        env=env, stdout=out, stderr=err)
                time.sleep(1)
                if xserver.poll() is not None:
                    raise RuntimeError("private X server exited before application launch")
            else:
                load("windows-ui").ensure_desktop()

            def accept_ready(index):
                process, profile, *_ = processes[index]
                diagnostic = wait_ready(process, profile)
                require_build(diagnostic, report["product_commit"])
                report["profiles"][index]["build_commit"] = diagnostic["build_commit"]
                report["profiles"][index]["first_start_setup"] = defer_setup(process, profile, env)
                report["observations"].append({"startup_pid": process.pid})

            for index, portable in launch_plan(os.name):
                profile = base / f"profile {index}"
                subprocess.run([sys.executable, str(ROOT / "tools/prepare-test-profile.py"),
                                "--build", str(build), "--profile", str(profile)],
                               check=True, capture_output=True)
                fixtures.seed(profile)
                boundary = PeerBoundary(profile)
                expected = fixtures.snapshot(profile)
                stdout = base / f"peer-two-{index}.stdout.log"
                stderr = base / f"peer-two-{index}.stderr.log"
                outputs.extend((path, evidence / path.name) for path in (stdout, stderr))
                outputs.append((profile / "opencpn.log",
                                evidence / f"peer-two-{index}.opencpn.log"))
                command = launch_arguments(exe, profile, portable=portable)
                with stdout.open("wb") as out, stderr.open("wb") as err:
                    process = subprocess.Popen(
                        command, env=env, stdout=out, stderr=err,
                        cwd=launch_working_directory(profile, portable=portable))
                processes.append((process, profile, boundary, expected, stdout, stderr))
                report["profiles"].append({"index": index, "pid": process.pid,
                                           "profile": str(profile),
                                           "launch_mode": "portable" if portable else "ordinary",
                                           "arguments": command[1:]})
                if os.name == "nt" and index == 1:
                    # Clear first-run UI before opening the second native app.
                    # Both remain live during all boundary observations below.
                    accept_ready(0)
            # Require both processes to be live after both independent profiles
            # have completed deferred initialization. Sequential mode tests do
            # not exercise this state.
            for index in range(1 if os.name == "nt" else 0, len(processes)):
                accept_ready(index)
            assert all(process.poll() is None for process, *_ in processes), "two live apps required"
            for index, (process, profile, boundary, expected, stdout, stderr) in enumerate(processes):
                observed = boundary.observe(process.pid)
                assert fixtures.snapshot(profile) == expected, "navigation fixture changed while both apps ran"
                assert_no_secrets(profile, stdout, stderr)
                report["observations"][index].update(observed)
            for process, profile, *_ in (reversed(processes) if os.name == "nt" else processes):
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
            if os.name == "nt":
                # Observe while both owned processes still exist. Cleanup below
                # must retain the original gate failure even if observation fails.
                report['windows_failure_state'] = []
                for process, profile, *_ in processes:
                    try:
                        state = windows_startup_failure(process, profile, load('windows-ui'))
                    except Exception as observation_error:
                        state = {'pid': process.pid, 'observation_error': type(observation_error).__name__}
                    report['windows_failure_state'].append(state)
            raise
        finally:
            failures = finalize_evidence(processes, xserver, outputs, evidence, report)
            if failures and sys.exc_info()[0] is None:
                raise RuntimeError("Peer evidence finalization failed; see finalization_errors")


def main(argv=None):
    parser = argparse.ArgumentParser()
    default_exe = ROOT / "build/xnav-install" / ("opencpn.exe" if os.name == "nt" else "bin/opencpn")
    parser.add_argument("--exe", type=Path, default=default_exe)
    parser.add_argument("--build", type=Path, default=ROOT / "build" /
                        ("xnav-windows" if os.name == "nt" else "xnav-linux"))
    parser.add_argument("--evidence", type=Path, default=ROOT / "evidence/local")
    parser.add_argument("--expected-commit")
    parser.add_argument("--compiled-input-receipt", type=Path)
    args = parser.parse_args(argv)
    try:
        run(args)
    except BaseException:
        # run() preserves the original exception for programmatic callers.
        # Never let Python render its message, cause or context to CI stderr:
        # those may contain the same credentials as the sanitized evidence.
        print("Peer containment gate failed; see sanitized evidence if available.",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
