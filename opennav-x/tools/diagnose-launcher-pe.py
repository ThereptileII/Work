#!/usr/bin/env python3
"""Retain an inert native PE resource-update reproducer; never run its target."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess

from windows_icon import embed_icon


def pe_summary(path):
    data = path.read_bytes()
    pe = struct.unpack_from('<I', data, 60)[0]
    machine, count, _, symbols, symbol_count, optional_size = struct.unpack_from('<HHIIIH', data, pe + 4)
    table = pe + 24 + optional_size
    sections = []
    for index in range(count):
        start = table + index * 40
        name = data[start:start + 8].rstrip(b'\0').decode('ascii')
        virtual_size, address, size, offset = struct.unpack_from('<IIII', data, start + 8)
        sections.append({'name': name, 'virtualSize': virtual_size, 'rva': address,
                         'rawSize': size, 'rawOffset': offset,
                         'sha256': hashlib.sha256(data[offset:offset + size]).hexdigest()})
    strings = symbols + 18 * symbol_count
    return {'size': len(data), 'sha256': hashlib.sha256(data).hexdigest(),
            'machine': machine, 'symbolOffset': symbols, 'symbolCount': symbol_count,
            'stringOffset': strings,
            'stringLength': struct.unpack_from('<I', data, strings)[0] if strings + 4 <= len(data) else None,
            'sections': sections}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.name != 'nt':
        raise SystemExit('This diagnostic requires native Windows')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    go = shutil.which('go')
    env = dict(os.environ, GOOS='windows', GOARCH='386', GO386='sse2',
               CGO_ENABLED='0', GOTOOLCHAIN='local', GOWORK='off', GOEXPERIMENT='', GOFLAGS='')
    commands = []

    def run(label, arguments, check=True):
        result = subprocess.run([go, *arguments], cwd=output, env=env,
                                capture_output=True, text=True, timeout=120)
        commands.append({'label': label, 'arguments': arguments, 'exitCode': result.returncode})
        (output / (label + '.log')).write_text(result.stdout + result.stderr, encoding='utf-8')
        print(label + ': ' + result.stdout + result.stderr, flush=True)
        if check and result.returncode:
            raise RuntimeError(label + ' failed')
        return result.returncode

    version = subprocess.check_output([go, 'env', 'GOVERSION'], text=True).strip()
    if version != 'go1.27.1':
        raise SystemExit('Pinned Go 1.27.1 required')
    (output / 'go.mod').write_text('module example.com/inert-pe-diagnostic\n\ngo 1.27.1\n', encoding='utf-8')
    (output / 'fixture.go').write_text('package main\nfunc main() {}\n', encoding='utf-8')
    (output / 'inspect.go').write_text('''package main
import ("debug/pe"; "fmt"; "os")
func main() {
 f, err := pe.Open(os.Args[1])
 if err != nil { fmt.Printf("debug/pe: %v\\n", err); os.Exit(1) }
 defer f.Close()
 fmt.Printf("header=%+v\\nsymbols=%d strings=%d\\n", f.FileHeader, len(f.COFFSymbols), len(f.StringTable))
 for _, s := range f.Sections { fmt.Printf("section=%+v\\n", s.SectionHeader) }
}
''', encoding='utf-8')
    run('build', ['build', '-mod=readonly', '-trimpath', '-buildvcs=true',
                  '-ldflags=-H=windowsgui', '-o', 'before.exe', 'fixture.go'])
    run('build-inspector', ['build', '-o', 'inspect.exe', 'inspect.go'])
    before = output / 'before.exe'
    after = output / 'after.exe'
    shutil.copyfile(before, after)
    report = {'scope': 'Inert PE diagnostic only; no application or release qualification',
              'commit': os.environ.get('GITHUB_SHA'), 'goVersion': version,
              'before': pe_summary(before), 'commands': commands}
    helper = Path(__file__).with_name('windows_icon.py')
    report['iconHelperSha256'] = hashlib.sha256(helper.read_bytes()).hexdigest()
    run('before-build-info', ['version', '-m', str(before)])
    before_inspect = subprocess.run([str(output / 'inspect.exe'), str(before)], capture_output=True, text=True)
    (output / 'before-pe.log').write_text(before_inspect.stdout + before_inspect.stderr, encoding='utf-8')
    icon = Path(__file__).resolve().parents[1] / 'resources/branding/skager.ico'
    try:
        embed_icon(after, icon.read_bytes())
        report['embedResult'] = 'passed'
    except Exception as error:
        report['embedResult'] = repr(error)
    report['after'] = pe_summary(after)
    after_info = run('after-build-info', ['version', '-m', str(after)], check=False)
    after_inspect = subprocess.run([str(output / 'inspect.exe'), str(after)], capture_output=True, text=True)
    (output / 'after-pe.log').write_text(after_inspect.stdout + after_inspect.stderr, encoding='utf-8')
    report['beforePeExitCode'] = before_inspect.returncode
    report['afterPeExitCode'] = after_inspect.returncode
    (output / 'diagnostic.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2), flush=True)
    return int(after_info != 0 or after_inspect.returncode != 0 or report['embedResult'] != 'passed')


if __name__ == '__main__':
    raise SystemExit(main())
