#!/usr/bin/env python3
"""Compile and exercise the actual patched Downloader against loopback TLS."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / "upstream/OpenCPN"
PATCH = ROOT / "patches/opencpn-5.12.4-download-trust.patch"
PAYLOAD = b"SCRUM211 trusted downloader payload\n"
SENTINEL = b"pre-existing destination\n"


def run(args, **kwargs):
    return subprocess.run(args, check=True, text=True, **kwargs)


def openssl(*args):
    run(["openssl", *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def make_ca(directory, name):
    key = directory / f"{name}.key"
    cert = directory / f"{name}.pem"
    openssl("req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "2",
            "-subj", f"/CN={name}", "-keyout", str(key), "-out", str(cert))
    return key, cert


def make_leaf(directory, name, ca_key, ca_cert, dns_name, expired=False):
    key = directory / f"{name}.key"
    csr = directory / f"{name}.csr"
    cert = directory / f"{name}.pem"
    ext = directory / f"{name}.ext"
    ext.write_text(f"subjectAltName=DNS:{dns_name}\nextendedKeyUsage=serverAuth\n",
                   encoding="utf-8")
    openssl("req", "-new", "-newkey", "rsa:2048", "-nodes", "-subj",
            f"/CN={dns_name}", "-keyout", str(key), "-out", str(csr))
    if not expired:
        openssl("x509", "-req", "-in", str(csr), "-CA", str(ca_cert),
                "-CAkey", str(ca_key), "-CAcreateserial", "-days", "1",
                "-extfile", str(ext), "-out", str(cert))
        return key, cert
    database = directory / "index.txt"
    database.write_text("", encoding="ascii")
    (directory / "serial").write_text("1000\n", encoding="ascii")
    (directory / "newcerts").mkdir()
    config = directory / "ca.cnf"
    config.write_text(f"""
[ca]
default_ca=local
[local]
database={database}
serial={directory / 'serial'}
new_certs_dir={directory / 'newcerts'}
certificate={ca_cert}
private_key={ca_key}
default_md=sha256
policy=policy
x509_extensions=server
[policy]
commonName=supplied
[server]
subjectAltName=DNS:{dns_name}
extendedKeyUsage=serverAuth
""", encoding="utf-8")
    openssl("ca", "-batch", "-config", str(config), "-in", str(csr),
            "-out", str(cert), "-startdate", "20200101000000Z",
            "-enddate", "20200102000000Z")
    return key, cert


def prepare_source(directory):
    for relative in ("model/src/downloader.cpp", "model/src/plugin_handler.cpp",
                     "model/include/model/downloader.h"):
        destination = directory / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(UPSTREAM / relative, destination)
    subprocess.run(["patch", "-p1", "-d", str(directory)], check=True,
                   input=PATCH.read_bytes(), stdout=subprocess.DEVNULL)


def compile_probe(directory):
    binary = directory / "downloader-trust-probe"
    flags = subprocess.check_output(
        ["pkg-config", "--cflags", "--libs", "libcurl"], text=True).split()
    run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
         "-DOPENNAV_DOWNLOADER_TLS_TEST",
         "-I", str(ROOT / "tools/downloader-trust-shim"),
         "-I", str(directory / "model/include"),
         str(directory / "model/src/downloader.cpp"),
         str(ROOT / "tools/downloader-trust-probe.cpp"), *flags,
         "-o", str(binary)])
    return binary


def start_server(directory, name, cert, key):
    port_file = directory / f"{name}.port"
    process = subprocess.Popen(
        [sys.executable, str(ROOT / "tools/downloader-trust-server.py"),
         str(cert), str(key), str(port_file)], stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE, text=True)
    for _ in range(100):
        if port_file.exists():
            return process, json.loads(port_file.read_text())["port"]
        if process.poll() is not None:
            raise RuntimeError(process.stderr.read())
        time.sleep(0.02)
    process.terminate()
    raise RuntimeError("TLS fixture did not publish its port")


def probe(binary, directory, name, url, ca_file, expected_ok,
          expected_payload=None, cwd=None, reject_stream=False):
    destination = directory / f"{name}.output"
    destination.write_bytes(SENTINEL)
    env = dict(os.environ, OPENNAV_DOWNLOADER_TEST_CA_FILE=str(ca_file))
    args = [str(binary), url, str(destination)]
    if reject_stream:
        args.append("--reject-stream")
    result = subprocess.run(args, env=env,
                            text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, cwd=cwd)
    fields = dict(line.split("=", 1) for line in result.stdout.splitlines()
                  if "=" in line)
    actual_ok = result.returncode == 0 and fields.get("download_ok") == "true"
    if actual_ok != expected_ok:
        raise AssertionError(f"{name}: unexpected result\n{result.stdout}\n{result.stderr}")
    expected = expected_payload if expected_ok else SENTINEL
    if destination.read_bytes() != expected:
        raise AssertionError(f"{name}: destination was accepted or modified incorrectly")
    head_error = int(fields["head_error"])
    if expected_ok and head_error != 0:
        raise AssertionError(f"{name}: trusted HEAD unexpectedly failed")
    if not expected_ok and name != "partial_transfer" and not reject_stream and head_error == 0:
        raise AssertionError(f"{name}: HEAD accepted a rejected transport")
    if expected_ok and int(fields["head_size"]) != len(PAYLOAD):
        raise AssertionError(f"{name}: HEAD size did not match known body")
    if not expected_ok and fields.get("download_error") in (None, "0"):
        raise AssertionError(f"{name}: failure has no CURL error")
    if list(directory.glob(".ocpn-download-*")):
        raise AssertionError(f"{name}: partial staging file was retained")
    return {"case": name, "accepted": actual_ok,
            "downloadError": int(fields["download_error"]),
            "headError": int(fields["head_error"])}


def main():
    with tempfile.TemporaryDirectory(prefix="scrum211-downloader-") as raw:
        directory = Path(raw)
        source = directory / "source"
        prepare_source(source)
        binary = compile_probe(source)
        ca_key, ca_cert = make_ca(directory, "trusted-ca")
        other_key, other_cert = make_ca(directory, "untrusted-ca")
        valid_key, valid_cert = make_leaf(directory, "valid", ca_key, ca_cert, "localhost")
        wrong_key, wrong_cert = make_leaf(directory, "wrong", ca_key, ca_cert, "wrong.invalid")
        untrusted_key, untrusted_cert = make_leaf(
            directory, "untrusted", other_key, other_cert, "localhost")
        expired_key, expired_cert = make_leaf(
            directory, "expired", ca_key, ca_cert, "localhost", expired=True)
        servers = []
        results = []
        try:
            valid_server, valid_port = start_server(directory, "valid", valid_cert, valid_key)
            servers.append(valid_server)
            for path, name, ok, body in (
                    ("/payload", "valid", True, PAYLOAD),
                    ("/redirect", "https_redirect", True, PAYLOAD),
                    ("/downgrade", "http_downgrade", False, None),
                    ("/local-file", "local_file_redirect", False, None),
                    ("/partial", "partial_transfer", False, None)):
                results.append(probe(binary, directory, name,
                    f"https://localhost:{valid_port}{path}", ca_cert, ok, body))
            for name, cert, key in (("wrong_host", wrong_cert, wrong_key),
                                    ("untrusted", untrusted_cert, untrusted_key),
                                    ("expired", expired_cert, expired_key)):
                server, port = start_server(directory, name, cert, key)
                servers.append(server)
                results.append(probe(binary, directory, name,
                    f"https://localhost:{port}/payload", ca_cert, False))
            results.append(probe(binary, directory, "write_exception",
                f"https://localhost:{valid_port}/payload", ca_cert, False,
                reject_stream=True))
            alternate_cwd = directory / "unrelated-cwd"
            alternate_cwd.mkdir()
            results.append(probe(binary, directory, "valid_different_cwd",
                f"https://localhost:{valid_port}/payload", ca_cert, True,
                PAYLOAD, alternate_cwd))
            results.append(probe(binary, directory, "initial_http",
                                 "http://localhost:9/payload", ca_cert, False))
            results.append(probe(binary, directory, "missing_trust",
                f"https://localhost:{valid_port}/payload",
                directory / "missing-ca.pem", False))
        finally:
            for server in servers:
                server.terminate()
            for server in servers:
                server.wait(timeout=5)
        print(json.dumps({"status": "passed", "cases": results}, indent=2))


if __name__ == "__main__":
    main()
