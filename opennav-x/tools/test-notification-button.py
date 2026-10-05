#!/usr/bin/env python3
"""Small isolated paint/cache/blend checks; does not build or launch OpenCPN."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def body(text, start):
    begin = text.index(start)
    end = text.index('{', begin) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[begin:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--notification-source', type=Path, required=True)
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--wx-prefix', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = args.notification_source.read_text()
    # Exact production entry branch; unmodified stock drawing is outside this
    # fixture. Full production translation-unit compilation is a separate check.
    create = source[source.index('void NotificationButton::CreateBmp('):]
    create = create[:create.index('  // wxString gpsIconName;')]
    create = create.replace('{\n', '{\n  ++builds;\n', 1)
    create += '  ++fallbacks; m_lastNoteIconName = m_NoteIconName;\n}\n'
    methods = '\n'.join(body(source, start) for start in (
        'bool NotificationButton::UpdateStatus(',
        'void NotificationButton::SetColorScheme(',
        'wxRect NotificationButton::GetLogicalRect('))
    (out / 'notification-methods.inc').write_text(methods + '\n' + create)
    wx = [str(args.wx_prefix / 'bin/wx-config'), '--prefix=' + str(args.wx_prefix)]
    flags = shlex.split(subprocess.check_output(wx + ['--cxxflags', '--libs', 'core,base'], text=True))
    for name in ('bitmap', 'cache', 'gl'):
        command = ['c++', '-std=c++17', '-DOPENNAV_X', '-DocpnUSE_GL',
                   '-I' + str(ROOT / 'src'), '-I' + str(out),
                   '-I' + str(args.upstream / 'libs/s52plib/src'),
                   str(ROOT / f'tests/notification_button_{name}_test.cpp')]
        if name != 'gl':
            command += flags + ['-Wl,-rpath,' + str(args.wx_prefix / 'lib')]
        result = subprocess.run(command + ['-o', str(out / name)], capture_output=True, text=True)
        (out / f'{name}-compile.log').write_text(result.stdout + result.stderr)
        result.check_returncode()
    env = dict(os.environ, LD_LIBRARY_PATH=str(args.wx_prefix / 'lib'),
               GDK_BACKEND='x11', GSETTINGS_BACKEND='memory', NO_AT_BRIDGE='1')
    env.pop('WAYLAND_DISPLAY', None)
    display = next(n for n in range(310, 400) if not Path(f'/tmp/.X{n}-lock').exists())
    env['DISPLAY'] = f':{display}'
    server = subprocess.Popen([str(args.wx_prefix / 'bin/Xvfb'), env['DISPLAY'],
        '-screen', '0', '800x600x24', '-nolisten', 'tcp'], env=env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(.4)
        assert server.poll() is None
        logs = []
        for name in ('bitmap', 'cache', 'gl'):
            command = [str(out / name)]
            if name == 'bitmap': command += [str(out / 'bitmap-fixture.png')]
            result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=20)
            logs.append(result.stdout + result.stderr)
            (out / 'checks.log').write_text(''.join(logs))
            print(logs[-1], end='')
            result.check_returncode()
    finally:
        server.terminate()
        server.wait(timeout=5)


if __name__ == '__main__':
    main()
