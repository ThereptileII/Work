#!/usr/bin/env python3
"""Run the shared offline boat-feedback components, never the installed product.

Build only the boat_feedback_tests target, then pass its generated manifest.
Native Windows runs are restricted to disposable CI, as for component captures.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
TESTS = {
    'chart_info_tests', 'pilot_status_tests', 'anchor_route_transition_tests',
    'navigation_naming_tests', 'route_context_tests', 'chart_info_drawer_test',
    'navigation_name_editor_test', 'route_context_card_test',
    'chart_light_hover_tests', 'ais_drawer_scroll_test', 'online_ais_radius_test',
    'chart_anchor_watch_renderer_test',
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--runtime-dir', action='append', type=Path, default=[])
    parser.add_argument('--expected-commit', help='CI checkout commit; defaults to current checkout')
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding='utf-8'))
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    expected = args.expected_commit or commit
    if not re.fullmatch('[0-9a-f]{40}', expected) or manifest.get('commit') != expected or commit != expected:
        raise RuntimeError('Configured manifest, requested commit and checkout must match; reconfigure the focused targets')
    entries = manifest.get('tests', [])
    names = [entry.get('name') for entry in entries]
    if manifest.get('schema') != 1 or len(names) != len(TESTS) or set(names) != TESTS:
        raise RuntimeError(f'Feedback manifest must contain each of the {len(TESTS)} expected tests exactly once')
    windows = sys.platform == 'win32'
    if windows and os.environ.get('GITHUB_ACTIONS') != 'true':
        raise RuntimeError('Windows component tests require disposable CI, never the boat desktop')
    for entry in entries:
        path = Path(entry['path']).resolve()
        if not path.is_file() or path.name != entry['name'] + ('.exe' if windows else ''):
            raise RuntimeError(f"Build the missing or incorrectly named component: {entry['name']}")
        entry['path'] = str(path)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ)
    env.pop('AISSTREAM_API_KEY', None)
    runtime = [str(path.resolve()) for path in args.runtime_dir]
    runtime.extend(sorted({str(Path(entry['path']).parent) for entry in entries}))
    env['PATH'] = os.pathsep.join(runtime + [env.get('PATH', '')])
    if not windows:
        env.pop('WAYLAND_DISPLAY', None)
        env['GDK_BACKEND'] = 'x11'
        if not shutil.which('xvfb-run'):
            raise RuntimeError('Offline widget checks require xvfb-run for an isolated desktop')
    dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).strip())
    result = {
        'schema': 1, 'source_commit': commit, 'source_dirty': dirty,
        'platform': sys.platform, 'manifest_sha256': hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
        'scope': 'Offline production component libraries with owned fixtures; no product/profile/chart/network/hardware acceptance',
        'passed': True, 'tests': [],
    }
    for entry in entries:
        name, path = entry['name'], Path(entry['path'])
        command = [str(path)]
        if not windows:
            command = ['xvfb-run', '-a', '-s', '-screen 0 1280x800x24 -nolisten tcp'] + command
        started = time.monotonic()
        timed_out = False
        with subprocess.Popen(command, cwd=output, env=env, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, start_new_session=not windows) as process:
            try:
                stdout, stderr = process.communicate(timeout=60)
                code = process.returncode
            except subprocess.TimeoutExpired:
                timed_out = True
                # Kill only this owned test group, including its Xvfb wrapper.
                if windows:
                    process.kill()
                else:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                stdout, stderr = process.communicate()
                code = None
        (output / (name + '.stdout.log')).write_bytes(stdout)
        (output / (name + '.stderr.log')).write_bytes(stderr)
        passed = code == 0 and not timed_out
        result['passed'] &= passed
        result['tests'].append({
            'name': name, 'passed': passed, 'exit_code': code, 'timed_out': timed_out,
            'elapsed_seconds': round(time.monotonic() - started, 3),
            'binary_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        })
        print(f'{name}: {"passed" if passed else "FAILED"}', flush=True)
    (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
