#!/usr/bin/env python3
"""Build actual patched wxCurlHTTP and exercise it against owned TLS peers."""
import json
import os
from pathlib import Path
import shutil
import shlex
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / "upstream/OpenCPN"
PATCH = ROOT / "patches/opencpn-5.12.4-wxcurl-trust.patch"
PAYLOAD_SIZE = len(b"SCRUM211 wxCurlHTTP trusted payload\n")


def run(args, **kwargs):
    return subprocess.run(args, check=True, text=True, **kwargs)


def openssl(*args):
    run(["openssl", *args], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def make_ca(directory, name):
    key, cert = directory / f"{name}.key", directory / f"{name}.pem"
    openssl("req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "2",
            "-subj", f"/CN={name}", "-keyout", str(key), "-out", str(cert))
    return key, cert


def make_leaf(directory, name, ca_key, ca_cert, dns_name, expired=False):
    key, csr, cert = (directory / f"{name}.{suffix}"
                      for suffix in ("key", "csr", "pem"))
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
    source = directory / "source"
    shutil.copytree(UPSTREAM / "libs/wxcurl", source / "libs/wxcurl")
    subprocess.run(["patch", "-p1", "-d", str(source)], check=True,
                   input=PATCH.read_bytes(), stdout=subprocess.DEVNULL)
    return source


def compile_probe(source, output):
    wx_config = os.environ.get("WX_CONFIG", "wx-config")
    wx_args = [wx_config]
    if os.environ.get("WX_CONFIG_PREFIX"):
        wx_args.append(f"--prefix={os.environ['WX_CONFIG_PREFIX']}")
    wx_flags = shlex.split(subprocess.check_output(
        [*wx_args, "--cxxflags"], text=True))
    wx_libs = shlex.split(subprocess.check_output(
        [*wx_args, "--libs", "core,base"], text=True))
    curl_flags = shlex.split(subprocess.check_output(
        ["pkg-config", "--cflags", "--libs", "libcurl"], text=True))
    compiler = os.environ.get("CXX", "c++")
    run([compiler, "-std=c++17", "-DOPENNAV_WXCURL_TLS_TEST",
         "-I", str(source / "libs/wxcurl/include"), *wx_flags,
         str(source / "libs/wxcurl/src/base.cpp"),
         str(source / "libs/wxcurl/src/http.cpp"),
         str(ROOT / "tools/wxcurl-trust-probe.cpp"), *wx_libs, *curl_flags,
         "-o", str(output)])


def start_server(directory, name, cert, key, mode="tls", redirect_target=""):
    port_file = directory / f"{name}.port"
    command = [sys.executable, str(ROOT / "tools/wxcurl-trust-server.py"),
               mode, str(cert), str(key), str(port_file)]
    if redirect_target:
        command.append(redirect_target)
    process = subprocess.Popen(
        command, stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE, text=True)
    for _ in range(100):
        if port_file.exists():
            return process, json.loads(port_file.read_text())["port"]
        if process.poll() is not None:
            raise RuntimeError(process.stderr.read())
        time.sleep(0.02)
    process.terminate()
    raise RuntimeError("TLS fixture did not publish its port")


def probe(binary, name, url, ca_file, accepted, cwd=None):
    env = dict(os.environ, OPENNAV_WXCURL_TEST_CA_FILE=str(ca_file))
    result = subprocess.run([str(binary), url], env=env, cwd=cwd, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    fields = dict(line.split("=", 1) for line in result.stdout.splitlines()
                  if "=" in line)
    if fields.get("bad_option_blocked") != "true":
        raise AssertionError(f"{name}: failed setopt did not block Perform")
    actual = (result.returncode == 0 and fields.get("get_ok") == "true"
              and fields.get("head_ok") == "true")
    if actual != accepted:
        raise AssertionError(f"{name}: unexpected result\n{result.stdout}\n{result.stderr}")
    if accepted and int(fields["get_bytes"]) != PAYLOAD_SIZE:
        raise AssertionError(f"{name}: actual wxCurlHTTP body size differs")
    if not accepted and (fields.get("get_ok") != "false"
                         or fields.get("head_ok") != "false"):
        raise AssertionError(f"{name}: GET or HEAD was not rejected")
    return {"case": name, "accepted": actual,
            "getError": fields.get("get_error"),
            "headError": fields.get("head_error")}


def configure_only(binary, scheme):
    result = subprocess.run([str(binary), "--configure", f"{scheme}://localhost/"],
                            text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, env=os.environ)
    if result.returncode or "configure_ok=true" not in result.stdout:
        raise AssertionError(f"{scheme}: WXCURL setup rejected supported initial protocol")
    return {"case": f"{scheme}_configuration", "accepted": True}


def main():
    with tempfile.TemporaryDirectory(prefix="scrum211-wxcurl-") as raw:
        directory = Path(raw)
        source = prepare_source(directory)
        binary = directory / "wxcurl-trust-probe"
        compile_probe(source, binary)
        ca_key, ca_cert = make_ca(directory, "trusted-ca")
        other_key, other_cert = make_ca(directory, "untrusted-ca")
        valid_key, valid_cert = make_leaf(directory, "valid", ca_key, ca_cert, "localhost")
        wrong_key, wrong_cert = make_leaf(directory, "wrong", ca_key, ca_cert, "wrong.invalid")
        untrusted_key, untrusted_cert = make_leaf(
            directory, "untrusted", other_key, other_cert, "localhost")
        expired_key, expired_cert = make_leaf(
            directory, "expired", ca_key, ca_cert, "localhost", expired=True)
        servers, results = [], []
        try:
            server, port = start_server(directory, "valid", valid_cert, valid_key)
            servers.append(server)
            for path, name, accepted in (("/payload", "valid", True),
                                         ("/redirect", "https_redirect", True),
                                         ("/downgrade", "http_downgrade", False),
                                         ("/local-file", "local_file_redirect", False)):
                results.append(probe(binary, name,
                    f"https://localhost:{port}{path}", ca_cert, accepted))
            unrelated = directory / "unrelated-cwd"
            unrelated.mkdir()
            results.append(probe(binary, "valid_different_cwd",
                f"https://localhost:{port}/payload", ca_cert, True, unrelated))
            plain_server, plain_port = start_server(
                directory, "plain-http", valid_cert, valid_key, "plain",
                f"https://localhost:{port}/payload")
            servers.append(plain_server)
            results.append(probe(binary, "plain_http",
                f"http://localhost:{plain_port}/payload", ca_cert, True))
            results.append(probe(binary, "http_to_https",
                f"http://localhost:{plain_port}/to-https", ca_cert, True))
            results.extend(configure_only(binary, scheme)
                           for scheme in ("ftp", "telnet"))
            for name, cert, key in (("wrong_host", wrong_cert, wrong_key),
                                    ("untrusted", untrusted_cert, untrusted_key),
                                    ("expired", expired_cert, expired_key)):
                case_server, case_port = start_server(directory, name, cert, key)
                servers.append(case_server)
                results.append(probe(binary, name,
                    f"https://localhost:{case_port}/payload", ca_cert, False))
            results.append(probe(binary, "missing_trust",
                f"https://localhost:{port}/payload", directory / "missing.pem", False))
        finally:
            for server in servers:
                server.terminate()
            for server in servers:
                server.wait(timeout=5)
        print(json.dumps({"status": "passed", "cases": results}, indent=2))


if __name__ == "__main__":
    main()
