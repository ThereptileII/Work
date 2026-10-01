"""Bounded native-Windows probe of curl's patched certificate generator.

This diagnostic proves only that the verified curl 8.22.0 generator can run
under the selected MSYS2 Perl and OpenSSL executable. It is not package or
curl-suite qualification.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import tarfile


ARCHIVE_SHA256 = "f7ef3ae8a22e521f289803fe93543eb64c329b58aa73a9e224dfd915a2a5f4f7"
ARCHIVE_BYTES = 2_953_092
OUTPUT_LIMIT = 65_536


def digest(path: Path) -> str:
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


def load_helpers():
    host_path = Path(__file__).with_name("test-curl-test-host.py")
    spec = importlib.util.spec_from_file_location("curl_test_host", host_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Cannot load the bounded native process helper")
    host = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(host)
    patch_path = Path(__file__).with_name("patch-curl-test-openssl.py")
    patch_spec = importlib.util.spec_from_file_location("curl_patch_test_openssl", patch_path)
    if patch_spec is None or patch_spec.loader is None:
        raise RuntimeError("Cannot load the hash-locked curl patch helper")
    patch = importlib.util.module_from_spec(patch_spec)
    patch_spec.loader.exec_module(patch)
    return host, patch, host_path, patch_path


def locked_member_hash(archive: Path, relative: str) -> str:
    member_name = "curl-8.22.0/" + relative
    with tarfile.open(archive, mode="r:xz") as source:
        member = source.getmember(member_name)
        if not member.isfile() or member.size > 1_000_000:
            raise RuntimeError(f"Unsafe source archive member: {relative}")
        stream = source.extractfile(member)
        if stream is None:
            raise RuntimeError(f"Cannot read source archive member: {relative}")
        data = stream.read(1_000_001)
    if len(data) != member.size or len(data) > 1_000_000:
        raise RuntimeError(f"Source archive member size differs: {relative}")
    return hashlib.sha256(data).hexdigest()


def checked_invoke(host, command, cwd: Path, env: dict[str, str], output: Path,
                   deadline: int, label: str) -> dict:
    result = host.invoke(command, cwd, env, output, deadline=deadline)
    sanitized_lines = []
    for line in result["output"].splitlines(keepends=True):
        if line.startswith("PATH used:"):
            line = "PATH used: [redacted]\n"
        sanitized_lines.append(line)
    safe_output = "".join(sanitized_lines)[:OUTPUT_LIMIT]
    output.write_text(safe_output, encoding="utf-8")
    if result["violation"] or result["exitCode"] != 0:
        raise RuntimeError(f"{label} failed: {result['violation'] or result['exitCode']}")
    return {key: value for key, value in result.items() if key != "output"} | {
        "output": safe_output
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", required=True, type=Path,
                        help="existing evidence/local/curl-test-host directory")
    parser.add_argument("--msys-perl", required=True, type=Path)
    parser.add_argument("--openssl", required=True, type=Path,
                        help="native Windows OpenSSL executable to select explicitly")
    args = parser.parse_args()
    if os.name != "nt":
        raise SystemExit("Requires native Windows; Linux is not acceptance")

    evidence = args.evidence.resolve(strict=True)
    report_path = evidence / "msys-certificate-probe.json"
    report = {
        "schemaVersion": 1,
        "purpose": "MSYS2 curl certificate generator diagnostic only",
        "passed": False,
        "source": {"sha256": ARCHIVE_SHA256, "bytes": ARCHIVE_BYTES},
    }
    try:
        archive = evidence / "curl-8.22.0.tar.xz"
        source_root = evidence / "curl-8.22.0"
        tests = source_root / "tests"
        certs = tests / "certs"
        genserv = certs / "genserv.pl"
        for required in (archive, genserv, certs / "test-ca.prm",
                         certs / "test-ca.cnf", certs / "test-localhost.prm"):
            if not required.is_file() or required.is_symlink():
                raise RuntimeError(f"Verified curl input is missing or unsafe: {required.name}")
        if archive.stat().st_size != ARCHIVE_BYTES or digest(archive) != ARCHIVE_SHA256:
            raise RuntimeError("Existing curl source archive differs from the reviewed lock")

        perl = args.msys_perl.resolve(strict=True)
        openssl = args.openssl.resolve(strict=True)
        if perl.suffix.lower() != ".exe" or openssl.suffix.lower() != ".exe":
            raise RuntimeError("Selected Perl and OpenSSL must be native Windows executables")
        if not openssl.is_file() or not perl.is_file():
            raise RuntimeError("Selected Perl or OpenSSL is not a regular executable file")

        host, patch_helper, host_script, patch_script = load_helpers()
        msys_runtime = perl.parent / "msys-2.0.dll"
        if not msys_runtime.is_file() or msys_runtime.is_symlink():
            raise RuntimeError("Selected Perl is not accompanied by the MSYS2 runtime DLL")
        report["tools"] = {
            "perl": {"path": str(perl), "sha256": digest(perl),
                     "runtimeDll": str(msys_runtime), "runtimeDllSha256": digest(msys_runtime)},
            "openssl": {"path": str(openssl), "sha256": digest(openssl)},
            "controllerSha256": digest(host_script),
            "patchHelperSha256": digest(patch_script),
        }
        env = os.environ.copy()
        # Resolve the generator's bare `openssl` command to this explicit tool.
        env["PATH"] = str(openssl.parent) + os.pathsep + str(perl.parent) + os.pathsep + env.get("PATH", "")
        with tempfile.TemporaryDirectory(prefix="curl-msys-cert-", dir=evidence) as temporary:
            work = Path(temporary)
            version = checked_invoke(host, [str(openssl), "version"], work, env,
                                    evidence / "msys-certificate-openssl-version.txt", 15,
                                    "selected OpenSSL version probe")
            version_line = next((line.strip() for line in version["output"].splitlines() if line.strip()), "")
            if not version_line.startswith("OpenSSL "):
                raise RuntimeError("Selected executable did not report an OpenSSL version")
            perl_version = checked_invoke(host, [str(perl), "-v"], work, env,
                                          evidence / "msys-certificate-perl-version.txt", 15,
                                          "selected Perl version probe")
            perl_os = checked_invoke(host, [str(perl), "-e", "print $^O"], work, env,
                                     evidence / "msys-certificate-perl-os.txt", 15,
                                     "selected Perl host probe")
            perl_os_name = perl_os["output"].strip()
            if perl_os_name not in ("cygwin", "msys"):
                raise RuntimeError(f"Selected Perl does not report a POSIX host: {perl_os_name}")
            report["tools"]["openssl"]["version"] = version_line[:512]
            report["tools"]["perl"]["versionOutput"] = perl_version["output"][:2048]
            report["tools"]["perl"]["host"] = perl_os_name

            original_hash = digest(genserv)
            source_members = ("tests/certs/test-ca.prm", "tests/certs/test-ca.cnf",
                              "tests/certs/test-localhost.prm")
            source_hashes = {name: digest(certs / Path(name).name) for name in source_members}
            for name, value in source_hashes.items():
                if locked_member_hash(archive, name) != value:
                    raise RuntimeError(f"Extracted curl certificate input differs from locked archive: {name}")
            patch_receipt = patch_helper.patch_source(genserv)
            patched_hash = digest(genserv)
            if patched_hash != patch_helper.PATCHED_SHA256:
                raise RuntimeError("Hash-locked genserv patch did not produce the reviewed bytes")
            report["module"] = {
                "path": "tests/certs/genserv.pl",
                "beforeSha256": original_hash,
                "afterSha256": patched_hash,
                "certificateInputs": source_hashes,
                "patch": patch_receipt,
            }
            report["opensslVersionProbe"] = version
            report["perlVersionProbe"] = perl_version

            result = checked_invoke(
                host,
                [str(perl), genserv.as_posix(), "test", "test-localhost.prm"],
                work, env, evidence / "msys-certificate-generation.txt", 60,
                "patched curl certificate generation",
            )
            report["generation"] = result
            selected = result["output"].splitlines()[0].strip()
            selected_path = checked_invoke(
                host, [str(perl.parent / "cygpath.exe"), "-w", selected],
                work, env, evidence / "msys-certificate-selected-tool.txt", 15,
                "generator-selected OpenSSL path conversion",
            )["output"].strip()
            selected_file = Path(selected_path)
            if not selected_file.is_file() and selected_file.suffix.lower() != ".exe":
                selected_file = Path(selected_path + ".exe")
            if selected_file.resolve(strict=True) != openssl or digest(selected_file) != digest(openssl):
                raise RuntimeError("Generator selected a different OpenSSL executable")
            report["tools"]["openssl"]["generatorSelectionVerified"] = True

            outputs = {
                name: work / name for name in (
                    "test-ca.cacert", "test-ca.key", "test-localhost.crt", "test-localhost.key"
                )
            }
            for name, path in outputs.items():
                if not path.is_file() or path.is_symlink() or path.stat().st_size <= 0:
                    raise RuntimeError(f"Certificate generator did not create {name}")
            report["generatedFiles"] = {
                name: {"bytes": path.stat().st_size, "sha256": digest(path)}
                for name, path in outputs.items()
            }

            verify_ca = checked_invoke(
                host, [str(openssl), "verify", "-CAfile", str(outputs["test-ca.cacert"]),
                       str(outputs["test-ca.cacert"])], work, env,
                evidence / "msys-certificate-verify-ca.txt", 15, "generated CA verification",
            )
            verify_host = checked_invoke(
                host, [str(openssl), "verify", "-CAfile", str(outputs["test-ca.cacert"]),
                       str(outputs["test-localhost.crt"])], work, env,
                evidence / "msys-certificate-verify-host.txt", 15, "generated host certificate verification",
            )
            key_checks = {}
            for name in ("test-ca.key", "test-localhost.key"):
                key_checks[name] = checked_invoke(
                    host, [str(openssl), "pkey", "-in", str(outputs[name]), "-check", "-noout"],
                    work, env, evidence / ("msys-certificate-key-" + name + ".txt"),
                    15, f"generated key check ({name})",
                )
            report["verification"] = {
                "ca": verify_ca,
                "hostCertificate": verify_host,
                "keys": key_checks,
            }
            report["passed"] = True
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
        return_code = 1
    else:
        return_code = 0
    finally:
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
