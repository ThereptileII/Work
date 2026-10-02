#!/usr/bin/env python3
"""Focused actual-source plugin download guard with real TLS and libarchive."""
import argparse
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tarfile
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
PIN = "37fd0cddb7334fe489e9f18aa163977a9c5c84f7"
EXPECTED = {
    "model/src/plugin_handler.cpp": "912c756ea9d1b17eb8411132feae7c0136345e327f79d522bae478a36a218471",
    "model/src/downloader.cpp": "6163ffeab05b71573e56c568e89616237817226ad2a65558e1f32ea2b7ad81ab",
    "model/include/model/downloader.h": "147a517339966a59375ba5af0ce4fec052b9fd3c35bdc05e54d2453d27682346",
}
SIGNATURES = (
    "static std::string dirListPath(",
    "std::string PluginHandler::FileListPath(",
    "std::string PluginHandler::VersionPath(",
    "static void saveFilelist(",
    "static void saveDirlist(",
    "static void saveVersion(",
    "static int copy_data(",
    "bool PluginHandler::ArchiveCheck(",
    "bool PluginHandler::ExplodeTarball(",
    "bool PluginHandler::ExtractTarball(",
    "bool PluginHandler::InstallPlugin(PluginMetadata plugin, std::string path)",
    "bool PluginHandler::InstallPlugin(PluginMetadata plugin)",
)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def prepare(upstream, source):
    for relative in EXPECTED:
        data = subprocess.check_output(["git", "-C", str(upstream), "show",
                                        f"{PIN}:{relative}"])
        dest = source / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
    return patch_and_slice(source)


def patch_and_slice(source, patch_program="patch"):
    patch = ROOT / "patches/opencpn-5.12.4-download-trust.patch"
    subprocess.run([str(patch_program), "--batch", "--fuzz=0", "-p1", "-d", str(source)],
                   input=patch.read_bytes(), check=True, stdout=subprocess.DEVNULL)
    for relative, expected in EXPECTED.items():
        if sha((source / relative).read_bytes()) != expected:
            raise AssertionError(f"reviewed patched source changed: {relative}")
    text = (source / "model/src/plugin_handler.cpp").read_text()
    slices = []
    records = []
    for signature in SIGNATURES:
        if text.count(signature) != 1:
            raise AssertionError(f"source boundary not unique: {signature}")
        start = text.index(signature)
        # The entire source file hash above fixes formatting and boundaries.
        # Pinned function closing braces alone begin at column zero.
        end = text.index("\n}", start) + 2
        body = text[start:end] + "\n"
        slices.append(body)
        records.append({"signature": signature, "firstLine": text.count("\n", 0, start) + 1,
                        "sha256": sha(body.encode())})
    (source / "plugin-handler-slices.inc").write_text("\n".join(slices), newline="\n")
    return {"upstreamCommit": PIN, "patchedSourceSha256": EXPECTED,
            "patchSha256": sha(patch.read_bytes()), "unchangedFunctionSlices": records,
            "probeSha256": sha((ROOT / "tools/plugin-download-guard-probe.cpp").read_bytes())}


def compile_probe(source):
    binary = source / "plugin-download-guard-probe"
    flags = shlex.split(subprocess.check_output(
        ["pkg-config", "--cflags", "--libs", "libcurl", "libarchive"], text=True))
    subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-DOPENNAV_DOWNLOADER_TLS_TEST",
                    "-I", str(ROOT / "tools/downloader-trust-shim"),
                    "-I", str(source / "model/include"), "-I", str(source),
                    str(source / "model/src/downloader.cpp"),
                    str(ROOT / "tools/plugin-download-guard-probe.cpp"),
                    *flags, "-o", str(binary)], check=True)
    return binary


def snapshot(root):
    return {str(p.relative_to(root)): sha(p.read_bytes())
            for p in sorted(root.rglob("*")) if p.is_file()}


def probe(binary, work, name, url, ca, expected_ok):
    root = work / name
    for child in ("installed", "records", "temporary"):
        (root / child).mkdir(parents=True)
    (root / "installed/existing.txt").write_bytes(b"existing installed payload\n")
    for suffix in ("files", "dirs", "version"):
        (root / "records" / f"fixture.{suffix}").write_bytes(b"existing record\n")
    before = snapshot(root)
    env = dict(os.environ)
    env.pop("OPENNAV_DOWNLOADER_TEST_CA_FILE", None)
    if ca is not None:
        env["OPENNAV_DOWNLOADER_TEST_CA_FILE"] = str(ca)
    else:
        env["PATH"] = str(binary.parent) + os.pathsep + str(Path(os.environ["SystemRoot"]) / "System32")
    result = subprocess.run([str(binary), url, str(root)], env=env,
        capture_output=True, text=True, timeout=30)
    (work / f"{name}.stdout").write_text(result.stdout)
    (work / f"{name}.stderr").write_text(result.stderr)
    fields = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    if result.returncode != (0 if expected_ok else 1) or fields.get("install_ok") != str(expected_ok).lower():
        raise AssertionError(f"{name}: unexpected install result: {result.returncode}; {result.stdout}; {result.stderr}")
    after = snapshot(root)
    if expected_ok:
        if fields.get("archive_opens") != "1" or fields.get("archive_entries") != "2":
            raise AssertionError("valid archive did not traverse actual extraction")
        if (root / "installed/existing.txt").read_bytes() != b"updated inert payload\n" or (root / "installed/new.txt").read_bytes() != b"new inert payload\n":
            raise AssertionError("valid archive payload not installed")
        expected_files = [str(root / "installed/existing.txt"), str(root / "installed/new.txt")]
        if (root / "records/fixture.files").read_text().splitlines() != expected_files:
            raise AssertionError("file list not published")
        if (root / "records/fixture.version").read_text() != "2\n":
            raise AssertionError("version not published")
        if (root / "records/fixture.dirs").read_text().strip() != f"share: {root / 'installed'}":
            raise AssertionError("directory record not published")
    else:
        if fields.get("archive_opens") != "0" or fields.get("archive_entries") != "0":
            raise AssertionError(f"{name}: failed download entered archive installation")
        if not fields.get("error", "").startswith("Cannot download plugin: Fixture [") or fields["error"].endswith("[]"):
            raise AssertionError(f"{name}: useful downloader error not propagated")
        if before != after:
            raise AssertionError(f"{name}: failed download changed installed files/records or retained temporary bytes")
    return {"case": name, "passed": True, "accepted": expected_ok,
            "archiveOpens": int(fields["archive_opens"]),
            "archiveEntries": int(fields["archive_entries"]),
            "installedFilesAndRecordsUnchanged": before == after,
            "error": fields["error"], "before": before, "after": after}


def start_server(work, name, cert, key, archive):
    port = work / f"{name}.port"
    with (work / f"{name}-server.log").open("w") as log:
        process = subprocess.Popen([sys.executable, str(ROOT / "tools/plugin-download-guard-server.py"),
            str(cert), str(key), str(port), str(archive)], stdout=log, stderr=log)
    for _ in range(100):
        if port.exists():
            return process, json.loads(port.read_text())["port"]
        if process.poll() is not None:
            raise RuntimeError(f"TLS fixture exited before publishing port; see {name}-server.log")
        time.sleep(0.02)
    process.terminate()
    process.wait(timeout=5)
    raise RuntimeError("TLS fixture did not start")


def run_cases(binary, output, ca, valid_cert, valid_key, bad_cert, bad_key, result):
    archive = output / "inert.tar"
    with tarfile.open(archive, "w", format=tarfile.USTAR_FORMAT) as tar:
        for name, data in (("fixture/metadata.xml", b"<plugin/>"),
            ("fixture/share/existing.txt", b"updated inert payload\n"),
            ("fixture/share/new.txt", b"new inert payload\n")):
            info = tarfile.TarInfo(name)
            info.size = len(data)
            info.mode = 0o600
            tar.addfile(info, io.BytesIO(data))
    result["archiveSha256"] = sha(archive.read_bytes())
    servers = []
    try:
        good, good_port = start_server(output, "trusted", valid_cert, valid_key, archive)
        servers.append(good)
        bad, bad_port = start_server(output, "untrusted", bad_cert, bad_key, archive)
        servers.append(bad)
        result["cases"] = []
        for name, url, expected in (
            ("valid", f"https://localhost:{good_port}/valid", True),
            ("untrusted_tls", f"https://localhost:{bad_port}/valid", False),
            ("interrupted_extractable_archive", f"https://localhost:{good_port}/interrupted", False)):
            result["cases"].append(probe(binary, output, name, url, ca, expected))
    finally:
        for process in servers:
            process.terminate()
        for process in servers:
            process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", type=Path, default=ROOT / "upstream/OpenCPN")
    parser.add_argument("--output", type=Path, required=True,
                        help="new ignored evidence directory (must not exist)")
    parser.add_argument("--prepare-only", action="store_true",
                        help="emit verified source slices for native build follow-up; run no tests")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    result = {"status": "failed", "scope": "actual source plugin download guard with fixture application routing",
              "limitations": ["Linux source-slice probe; not full installed PluginHandler or plugin ABI/UI acceptance",
                              "Fixture substitutes application path routing, metadata container, logging and temp allocation",
                              "Unexpected archive-error cleanup fails the probe; rollback is not qualified",
                              "Linux uses existing test-only CA and wx filesystem adapters; native Windows remains required"]}
    try:
        source = output / "source"
        result["source"] = prepare(args.upstream, source)
        if args.prepare_only:
            result["status"] = "prepared-not-tested"
            return
        binary = compile_probe(source)
        result["libraries"] = dict(zip(("libcurl", "libarchive"),
            subprocess.check_output(["pkg-config", "--modversion", "libcurl", "libarchive"],
                                    text=True).splitlines()))
        spec = importlib.util.spec_from_file_location("downloader_tls", ROOT / "tools/test-downloader-trust.py")
        tls = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tls)
        with tempfile.TemporaryDirectory(prefix="scrum211-plugin-ca-") as raw:
            certs = Path(raw)
            key, ca = tls.make_ca(certs, "trusted")
            other_key, other_ca = tls.make_ca(certs, "untrusted")
            valid_key, valid_cert = tls.make_leaf(certs, "valid", key, ca, "localhost")
            bad_key, bad_cert = tls.make_leaf(certs, "bad", other_key, other_ca, "localhost")
            run_cases(binary, output, ca, valid_cert, valid_key, bad_cert, bad_key, result)
        result["status"] = "passed"
    except Exception as error:
        result["failure"] = str(error)
        raise
    finally:
        (output / "results.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({"status": result["status"], "evidence": str(output / "results.json")}))


if __name__ == "__main__":
    main()
