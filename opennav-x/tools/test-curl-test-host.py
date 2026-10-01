"""Bounded native-Windows diagnostic for the locked curl Perl test host.

This does not qualify curl binaries, TLS, HTTP transfers, or the application.
It exercises upstream IPC readiness and upstream process launch separately.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
import urllib.request

ARCHIVE_SHA = "f7ef3ae8a22e521f289803fe93543eb64c329b58aa73a9e224dfd915a2a5f4f7"
ARCHIVE_BYTES = 2953092


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def invoke(command, cwd, env, output, deadline=15):
    with output.open("wb") as stream:
        process = subprocess.Popen(command, cwd=cwd, env=env, stdout=stream,
                                   stderr=subprocess.STDOUT)
        start = time.monotonic()
        violation = None
        while process.poll() is None:
            if time.monotonic() - start > deadline:
                violation = "deadline exceeded"
            elif output.stat().st_size > 65536:
                violation = "output limit exceeded"
            if violation:
                subprocess.run([str(Path(os.environ["SystemRoot"]) / "System32/taskkill.exe"),
                                "/PID", str(process.pid), "/T", "/F"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               timeout=10, check=True)
                process.wait(timeout=10)
                break
            time.sleep(0.05)
    with output.open("rb") as stream:
        data = stream.read(65536)
    output.write_bytes(data)
    return {"exitCode": process.returncode, "violation": violation,
            "elapsedMs": round((time.monotonic() - start) * 1000),
            "output": data.decode("utf-8", errors="replace")}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence", required=True, type=Path)
    parser.add_argument("--native-perl", required=True, type=Path)
    parser.add_argument("--msys-perl", required=True, type=Path)
    args = parser.parse_args()
    if os.name != "nt":
        raise SystemExit("Requires native Windows; Linux is not acceptance")
    evidence = args.evidence.resolve()
    evidence.mkdir(parents=True, exist_ok=False)
    report = {"schemaVersion": 1, "purpose": "test-host diagnostic only",
              "sourceSha256": ARCHIVE_SHA, "cases": [], "passed": False}
    try:
        archive = evidence / "curl-8.22.0.tar.xz"
        with urllib.request.urlopen("https://curl.se/download/curl-8.22.0.tar.xz", timeout=30) as reply:
            data = reply.read(ARCHIVE_BYTES + 1)
        archive.write_bytes(data)
        if len(data) != ARCHIVE_BYTES or digest(archive) != ARCHIVE_SHA:
            raise RuntimeError("Locked source archive mismatch")
        cmake = shutil.which("cmake.exe")
        if not cmake:
            raise RuntimeError("Native CMake required")
        extraction = invoke([cmake, "-E", "tar", "xf", str(archive)], evidence,
                            os.environ.copy(), evidence / "extract.txt", 45)
        if extraction["exitCode"] or extraction["violation"]:
            raise RuntimeError("Source extraction failed")
        tests = evidence / "curl-8.22.0/tests"
        report["modules"] = {name: digest(tests / name) for name in ("runner.pm", "servers.pm")}
        script = Path(__file__).with_suffix(".pl").resolve()
        for host, perl in (("native", args.native_perl), ("msys", args.msys_perl)):
            perl = perl.resolve(strict=True)
            runtime_sha = None
            if host == "msys":
                # MSYS2's packaged Perl reports cygwin in this runner image.
                # Bind its actual MSYS runtime as well as the Perl executable.
                runtime_sha = digest(perl.parent / "msys-2.0.dll")
            env = os.environ.copy()
            # A scoped host environment; it never affects later OpenSSL builds.
            env["PATH"] = str(perl.parent) + os.pathsep + env["PATH"]
            for operation in ("readiness", "server"):
                case_dir = evidence / (host + "-" + operation)
                case_dir.mkdir()
                result = invoke([str(perl), "-I", tests.as_posix(), script.as_posix(),
                                 operation, case_dir.as_posix()], evidence, env,
                                evidence / (host + "-" + operation + ".txt"))
                records = []
                for line in result["output"].splitlines():
                    if line.startswith("{"):
                        records.append(json.loads(line))
                row = {"host": host, "operation": operation, "perl": str(perl),
                       "perlSha256": digest(perl), "msysRuntimeSha256": runtime_sha,
                       "result": result, "records": records}
                report["cases"].append(row)
                if result["violation"] or result["exitCode"] or len(records) != 1:
                    raise RuntimeError("Diagnostic process failed: " + host + "/" + operation)
                value = records[0]
                expected_os = ("MSWin32",) if host == "native" else ("msys", "cygwin")
                if value["os"] not in expected_os:
                    raise RuntimeError("Unexpected Perl host: " + value["os"])
                if operation == "readiness":
                    if value["sent"] != 0 or value["response"][:2] != ["integrated", 0]:
                        raise RuntimeError("Upstream response was not delivered")
                    ready = "integrated" in value["ready"]
                    if ready != (host == "msys"):
                        raise RuntimeError("Readiness hypothesis contradicted")
                else:
                    started = value["marker"] and value["pid"] > 0
                    if bool(started) != (host == "msys"):
                        raise RuntimeError("Server launch hypothesis contradicted")
                    if host == "native" and "'exec' is not recognized" not in result["output"]:
                        raise RuntimeError("Expected native command failure not reproduced")
        report["passed"] = True
    finally:
        (evidence / "test-host.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
