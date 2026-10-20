#!/usr/bin/env python3
"""Execute the actual patched expanded-sector methods in a bounded wx/Mesa fixture."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("extract", ROOT / "tools/verify-anchor-loader.py")
extract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extract)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True, help="Pinned unpatched core source")
    parser.add_argument("--wx-prefix", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = args.source.resolve()
    original = source / "gui/src/s57chart.cpp"
    staged = out / "s57chart.cpp"
    staged.write_bytes(original.read_bytes())
    patch = ROOT / "patches/opencpn-5.12.4-chart-presentation.patch"
    parts = re.split(r"(?=^diff --git )", patch.read_text(), flags=re.M)
    for part in parts:
        if part.startswith("diff --git a/gui/src/s57chart.cpp "):
            result = subprocess.run(["patch", "--batch", "--forward", str(staged)],
                                    input=part, text=True, capture_output=True)
            with (out / "patch.log").open("a") as log:
                log.write(result.stdout + result.stderr)
            result.check_returncode()
    text = staged.read_text()
    processor = "bool s57_ProcessExtendedLightSectors("
    assert extract.function(text, processor) == extract.function(original.read_text(), processor)
    signatures = ["void s57_DrawExtendedLightSectors(", "void s57_DrawExtendedLightSectorsGL("]
    methods = {name: extract.function(text, name) for name in signatures}
    (out / "extended-methods.inc").write_text("".join(methods.values()))
    shaders = (source / "libs/s52plib/src/s52shaders.cpp").read_text()
    bodies = []
    for name in ["S52texture_2D_vertex_shader_source", "S52texture_2D_fragment_shader_source"]:
        start = shaders.index("static const GLchar *" + name + " =")
        match = re.search(r'^    "[^\n]*";\s*$', shaders[start:], re.M)
        assert match
        bodies.append(shaders[start:start + match.end()] + "\n")
    (out / "shaders.inc").write_text("".join(bodies))
    shared = (ROOT / "tests/chart_ca_fan_test.cpp").read_text()
    (out / "fixture-gl.inc").write_text("".join(extract.function(shared, signature) for signature in
        ["GLuint Program(", "wxImage ReadGL(", "void ClearGL(", "std::array<int,3> RGB(", "void Same("]))
    wx = [str(args.wx_prefix / "bin/wx-config"), "--prefix=" + str(args.wx_prefix)]
    flags = shlex.split(subprocess.check_output(wx + ["--cxxflags"], text=True))
    libs = shlex.split(subprocess.check_output(wx + ["--libs", "core,base,gl"], text=True))
    command = ["c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wno-unused-parameter",
               "-DOPENNAV_X", "-DocpnUSE_GL", *flags, "-I" + str(args.wx_prefix / "include"),
               "-I" + str(ROOT / "src"), "-I" + str(out),
               *["-I" + str(source / path) for path in
                 ["libs/s52plib/src", "libs/geoprim/src", "libs/pugixml", "libs/glu", "gui/include/gui"]],
               str(ROOT / "tests/chart_light_expanded_test.cpp"), *libs, "-lGL", "-lGLEW",
               "-o", str(out / "expanded-test")]
    result = subprocess.run(command, text=True, capture_output=True)
    (out / "compile.log").write_text(result.stdout + result.stderr)
    result.check_returncode()
    env = {**os.environ, "LD_LIBRARY_PATH": str(args.wx_prefix / "lib"), "GDK_BACKEND": "x11",
           "GSETTINGS_BACKEND": "memory", "NO_AT_BRIDGE": "1", "LIBGL_ALWAYS_SOFTWARE": "1"}
    env.pop("WAYLAND_DISPLAY", None)
    display = next(n for n in range(340, 400) if not Path(f"/tmp/.X{n}-lock").exists())
    env["DISPLAY"] = f":{display}"
    with (out / "xvfb.log").open("w") as log:
        server = subprocess.Popen([str(args.wx_prefix / "bin/Xvfb"), env["DISPLAY"], "-screen",
                                   "0", "1024x768x24", "-nolisten", "tcp"],
                                  env=env, stdout=log, stderr=log)
        try:
            time.sleep(.5)
            assert server.poll() is None
            result = subprocess.run([str(out / "expanded-test"), str(out)], env=env,
                                    text=True, capture_output=True, timeout=60)
        finally:
            server.terminate()
            server.wait(timeout=5)
    (out / "run.log").write_text(result.stdout + result.stderr)
    receipt = {"sourceSha256": sha(original), "patchSha256": sha(patch),
               "methods": {name: hashlib.sha256(body.encode()).hexdigest() for name, body in methods.items()},
               "nativeProcessorUnchanged": True,
               "helpers": {name: sha(ROOT / "src/integration" / name) for name in
                           ["ChartCaFan.h", "ChartLightHover.h", "ChartLightSymbol.h"]},
               "command": command, "exit": result.returncode, "stdout": result.stdout,
               "scope": "Actual sector methods and software/desktop GL paint; projection fixture, not chart/navigation or Windows acceptance."}
    (out / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(result.stdout + result.stderr)
    result.check_returncode()


if __name__ == "__main__":
    main()
