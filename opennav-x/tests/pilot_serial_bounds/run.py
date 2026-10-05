"""Prepare an isolated exact-pin source, apply the patch, build and run offline."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

PIN = "37fd0cddb7334fe489e9f18aa163977a9c5c84f7"
HERE = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument("--upstream", type=Path, required=True)
parser.add_argument("--build-dir", type=Path, required=True)
parser.add_argument("--sanitize", action="store_true")
args = parser.parse_args()
build = args.build_dir.resolve()
source = build / "source"
source.mkdir(parents=True, exist_ok=True)
files = ["model/src/comm_drv_n2k_serial.cpp", "libs/N2KParser/src/N2kMsg.cpp",
         "libs/N2KParser/include/N2kMsg.h", "libs/N2KParser/include/N2kDef.h"]
for name in files:
    target = source / name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(subprocess.check_output(
        ["git", "-C", str(args.upstream.resolve()), "show", f"{PIN}:{name}"]))

def run(*command, cwd=None):
    subprocess.run(list(map(str, command)), cwd=cwd, check=True)

# Independent tiny repository keeps git apply from discovering the parent repo.
run("git", "init", "-q", source)
patch = HERE.parents[1] / "patches/opencpn-5.12.4-pilot-serial.patch"
run("git", "apply", "--check", patch, cwd=source)
run("git", "apply", patch, cwd=source)
run("git", "apply", "--reverse", "--check", patch, cwd=source)
run("cmake", "-S", HERE, "-B", build / "build",
    f"-DOPENCPN_SOURCE_DIR:PATH={source.as_posix()}",
    f"-DPILOT_SERIAL_SANITIZE={'ON' if args.sanitize else 'OFF'}")
run("cmake", "--build", build / "build", "--config", "Debug", "--parallel", "2")
run("ctest", "--test-dir", build / "build", "-C", "Debug", "--output-on-failure")
(build / "evidence.json").write_text(json.dumps({
    "pin": PIN, "patch_apply_check": "passed", "tests": "passed",
    "patch_sha256": hashlib.sha256(patch.read_bytes()).hexdigest(),
    "patched_serial_sha256": hashlib.sha256((source / files[0]).read_bytes()).hexdigest(),
    "sanitizers": args.sanitize,
    "scope": "Exact extracted SendMessage and serial serializer; real N2kMsg; fake queue/listener. No serial I/O, worker, reconnect or permission qualification."
}, indent=2) + "\n")
