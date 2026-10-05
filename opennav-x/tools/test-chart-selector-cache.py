#!/usr/bin/env python3
"""Compile actual Piano atlas construction/cache code with captured GL uploads."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--piano-source', type=Path, required=True)
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--wx-prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expect-stale', action='store_true')
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    spec = importlib.util.spec_from_file_location(
        'selector_test', ROOT / 'tools/test-chart-selector.py')
    selector = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(selector)
    text = args.piano_source.read_text()
    own = (ROOT / 'src/integration/ChartPresentation.cpp').read_text()
    resolver = '\n'.join(selector.body(own, key) for key in (
        'wxColour Color(', 'bool ChartVectorSelectorInk('))
    (out / 'selector-resolver.inc').write_text(resolver + '\n')
    # Execute the original rebuild/retry branch; later drawing needs a GPU.
    prefix = text[text.index('void Piano::DrawGLSL('):]
    prefix = prefix[:prefix.index('  int y1 = off')] + '#endif\n}\n'
    methods = '\n'.join(selector.body(text, key) for key in (
        'void Piano::SetColorScheme(', 'void Piano::SyncChartPresentation(',
        'void Piano::BuildGLTexture(')) + '\n' + prefix
    (out / 'selector-methods.inc').write_text(methods)
    wx = [str(args.wx_prefix / 'bin/wx-config'),
          '--prefix=' + str(args.wx_prefix)]
    flags = shlex.split(subprocess.check_output(
        wx + ['--cxxflags', '--libs', 'core,base'], text=True))
    command = ['c++', '-std=c++17', '-DOPENNAV_X', '-DocpnUSE_GL',
               '-I' + str(ROOT / 'src'),
               '-I' + str(args.upstream / 'libs/s52plib/src'),
               '-I' + str(out), str(ROOT / 'tests/chart_selector_cache_test.cpp'),
               *flags, '-Wl,-rpath,' + str(args.wx_prefix / 'lib'),
               '-o', str(out / 'selector-cache')]
    result = subprocess.run(command, capture_output=True, text=True)
    (out / 'compile.log').write_text(result.stdout + result.stderr)
    result.check_returncode()
    env = {**os.environ, 'GDK_BACKEND': 'x11', 'GSETTINGS_BACKEND': 'memory',
           'NO_AT_BRIDGE': '1', 'LD_LIBRARY_PATH': str(args.wx_prefix / 'lib')}
    env.pop('WAYLAND_DISPLAY', None)
    display = 284
    while Path(f'/tmp/.X{display}-lock').exists():
        display += 1
    env['DISPLAY'] = ':' + str(display)
    server = subprocess.Popen(
        [str(args.wx_prefix / 'bin/Xvfb'), env['DISPLAY'], '-screen', '0',
         '420x200x24', '-nolisten', 'tcp'], env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(.4)
        if server.poll() is not None:
            raise RuntimeError('Private fixture X server failed to start')
        result = subprocess.run([str(out / 'selector-cache')], env=env,
                                capture_output=True, text=True, timeout=20)
    finally:
        server.terminate()
        server.wait(timeout=5)
    (out / 'checks.log').write_text(result.stdout + result.stderr)
    if args.expect_stale:
        expected = (result.returncode == 1 and
                    'lazy S52 color change must replace stale outline' in result.stderr)
    else:
        expected = result.returncode == 0
    (out / 'result.json').write_text(json.dumps({
        'scope': 'actual atlas construction/cache branch with GL upload capture; '
                 'not GPU/application/Windows acceptance',
        'sourceSha256': hashlib.sha256(args.piano_source.read_bytes()).hexdigest(),
        'extractedSha256': hashlib.sha256(methods.encode()).hexdigest(),
        'command': command, 'exitCode': result.returncode,
        'expectedStaleFailure': args.expect_stale, 'expectationMet': expected,
    }, indent=2) + '\n')
    print(result.stdout + result.stderr, end='')
    raise SystemExit(0 if expected else 1)


if __name__ == '__main__':
    main()
